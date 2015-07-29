/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

/* Regarding performance:
 *
 * Short story - STM32L4xx new flash ip. It works in 64 bit wide.
 *
 * The refernce manual for STM32FL476 is RM0351
 */

/* Erase time can be as high as 1000ms, 10x this and it's toast... */
#define FLASH_ERASE_TIMEOUT 10000
#define FLASH_WRITE_TIMEOUT 5

/* from rm 0351 */
#define FLASH_ACR	0x00
#define FLASH_PDKEYR	0x04
#define FLASH_KEYR	0x08
#define FLASH_OPTKEYR	0x0C
#define FLASH_SR	0x10
#define FLASH_CR	0x14
#define FLASH_ECR	0x18
#define FLASH_OPTR      0x20
#define FLASH_PCROP1SR	0x24
#define FLASH_PCROP1ER	0x28
#define FLASH_WRP1AR	0x2C
#define FLASH_WRP1BR	0x30
#define FLASH_PCROP2SR	0x44
#define FLASH_WRP2AR	0x4C
#define FLASH_WRP2BR	0x50

/* FLASH_ACR bits */
#define FLASH_ACR__LATENCY		(1<<0)
#define FLASH_ACR__PRFTEN		(1<<8)
#define FLASH_ACR__ICEN			(1<<9)
#define FLASH_ACR__DCEN			(1<<10)
#define FLASH_ACR__ICRST		(1<<11)
#define FLASH_ACR__DCRST		(1<<12)
#define FLASH_ACR__RUN_PD		(1<<13)
#define FLASH_ACR__SLEEP_PD		(1<<14)

/* FLASH_CR register bits */
#define FLASH_PG       (1 << 0)
#define FLASH_PER      (1 << 1)
#define FLASH_MER1     (1 << 2)
#define FLASH_PNB      (1 << 3)
#define FLASH_BKER     (1 << 11)
#define FLASH_MER2     (1 << 15)
#define FLASH_START    (1 << 16)
#define FLASH_OPTSTRT  (1 << 17)
#define FLASH_FSTPG    (1 << 18)
#define FLASH_EOPIE    (1 << 24)
#define FLASH_ERRIE    (1 << 25)
#define FLASH_RDERRIE  (1 << 26)
#define FLASH_OBL_LAUNCH (1 << 27)
#define FLASH_OPTLOCK  (1 << 30)
#define FLASH_LOCK     (1 << 31)

/* The sector number encoding */
#define FLASH_SNB(a)	(a << 3)

/* FLASH_SR register bits */
#define FLASH_BSY      (1 << 16) /* Operation in progres */
#define FLASH_OPTVERR  (1 << 15) /* Option validity error */
#define FLASH_RDERR    (1 << 14) /* Read protection error error */
#define FLASH_FASTERR  (1 << 9)  /* Fast programming error */
#define FLASH_MISERR   (1 << 8)  /* Fast programming data miss error */
#define FLASH_PGSERR   (1 << 7)  /* Programming sequence error */
#define FLASH_PGPERR   (1 << 6)  /* Programming parallelism error */
#define FLASH_PGAERR   (1 << 5)  /* Programming alignment error */
#define FLASH_WRPERR   (1 << 4)  /* Write protection error */
#define FLASH_PROGERR  (1 << 3)  /* Write protection error */
#define FLASH_OPERR    (1 << 1)  /* Operation error */
#define FLASH_EOP      (1 << 0)  /* End of operation */

#define FLASH_ERROR (FLASH_PROGERR | FLASH_PGSERR | FLASH_PGPERR | FLASH_PGAERR \
		     | FLASH_WRPERR | FLASH_OPERR | FLASH_OPTVERR | FLASH_RDERR | FLASH_FASTERR | FLASH_MISERR)

/* STM32_FLASH_OPTR register bits */
#define OPT_LOCK      (1 << 0)
#define OPT_START     (1 << 1)

/* STM32_FLASH_OBR bit definitions (reading) */
#define OPT_ERROR      0
#define OPT_READOUT    1
#define OPT_RDWDGSW    2
#define OPT_RDRSTSTOP  3
#define OPT_RDRSTSTDBY 4


/* register unlock keys */
#define KEY1           0x45670123
#define KEY2           0xCDEF89AB

/* option register unlock key */
#define OPTKEY1        0x08192A3B
#define OPTKEY2        0x4C5D6E7F

/* option bytes */
#define OPTION_BYTES_ADDRESS 0x1FFF7800

#define OPTION_BYTE_0_PR1 0x015500AA
#define OPTION_BYTE_0_PR0 0x01FF0011

#define FLASH_SECTOR_SIZE 2048

#define DBGMCU_IDCODE_REGISTER 0xE0042000
#define FLASH_BANK0_ADDRESS 0x08000000
#define FLASH_BASE 0x40022000                                        

#define BUFFER_SIZE 16384

#undef NO_OPTIONS

struct stm32l4x_rev {
	uint16_t rev;
	const char *str;
};

struct stm32x_options {
	uint8_t RDP;
	uint8_t protection;
	/* new zone */

	/* new cmd */
	uint8_t sram_erased_on_system_reset;
	uint8_t sram_parity_check;
	uint8_t flash_dual_bank;
	uint8_t flash_dual_bank_boot;
	uint8_t windows_watchdog_selection;
	uint8_t independent_watchdog_standby;
	uint8_t independent_watchdog_stop;
	uint8_t independent_watchdog_selection;
};

struct stm32l4x_part_info {
	uint16_t id;
	const char *device_str;
	const struct stm32l4x_rev *revs;
	size_t num_revs;
	unsigned int page_size;
	unsigned int pages_per_sector;
	uint16_t max_flash_size_kb;
	uint16_t first_bank_size_kb; /* used when has_dual_banks is true */
	uint16_t hole_sectors; /* use to recalculate the real sector number */

	uint32_t flash_base;	/* Flash controller registers location */
	uint32_t fsize_base;	/* Location of FSIZE register */
};

struct stm32l4x_flash_bank {
	int probed;
	uint32_t idcode;
	uint32_t user_bank_size;
	uint32_t flash_base;    /* Address of flash memory */
	struct stm32x_options option_bytes;

	/* Two zone of wpr by bank */
	uint32_t write_protected_a_start;
	uint32_t write_protected_a_end;

	uint32_t write_protected_b_start;
	uint32_t write_protected_b_end;
	const struct stm32l4x_part_info *part_info;
};

static const struct stm32l4x_rev stm32_415_revs[] = {
	{ 0x1000, "A" }, { 0x1001, "Z" }, { 0x1002, "Y" }, { 0x1003, "X" },
};

static const struct stm32l4x_rev stm32_435_revs[] = {
	{ 0x1000, "A" },
};

static const struct stm32l4x_part_info stm32l4x_parts[] = {
	{
	  .id				= 0x415,
	  .revs				= stm32_415_revs,
	  .num_revs			= ARRAY_SIZE(stm32_415_revs),
	  .device_str			= "STM32L4xx 1M",
	  .page_size			= 2048,
	  .max_flash_size_kb		= 1024,
	  .first_bank_size_kb           = 512,
	  .hole_sectors			= 0,
	  .flash_base			= 0x40022000,
	  .fsize_base			= 0x1FFF75E0,
	},
	{
	  .id				= 0x435,
	  .revs				= stm32_435_revs,
	  .num_revs			= ARRAY_SIZE(stm32_435_revs),
	  .device_str			= "STM32L4xx 256K",
	  .page_size			= 2048,
	  .max_flash_size_kb		= 256,
	  .first_bank_size_kb           = 128,
	  .hole_sectors			= 192,
	  .flash_base			= 0x40022000,
	  .fsize_base			= 0x1FFF75E0,
	},
	{
	  .id				= 0x452,/* TBV */
	  .revs				= stm32_435_revs,
	  .num_revs			= ARRAY_SIZE(stm32_435_revs),
	  .device_str			= "STM32L4xx 512K",
	  .page_size			= 2048,
	  .max_flash_size_kb		= 512,
	  .first_bank_size_kb           = 256,
	  .hole_sectors			= 128,
	  .flash_base			= 0x40022000,
	  .fsize_base			= 0x1FFF75E0,
	},
};

static int stm32x_unlock_reg(struct flash_bank *bank);
static int stm32x_probe(struct flash_bank *bank);

FLASH_BANK_COMMAND_HANDLER(stm32x_flash_bank_command)
{
	struct stm32l4x_flash_bank *stm32x_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	stm32x_info = malloc(sizeof(struct stm32l4x_flash_bank));
	bank->driver_priv = stm32x_info;

	stm32x_info->probed = 0;
	stm32x_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static inline int stm32x_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct stm32l4x_flash_bank *stm32x_info = bank->driver_priv;
	return target_read_u32(bank->target, stm32x_info->flash_base + FLASH_SR, status);
}

static int stm32x_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	struct stm32l4x_flash_bank *stm32x_info = bank->driver_priv;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = stm32x_get_flash_status(bank, &status);
		if (retval != ERROR_OK) {
			LOG_INFO("wait_status_busy, target_*_u32 : error : remote address 0x%x", stm32x_info->flash_base);
			return retval;
		}

		if ((status & FLASH_BSY) == 0)
			break;

		if (timeout-- <= 0) {
			LOG_INFO("wait_status_busy, time out expired");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	if (status & FLASH_WRPERR) {
		LOG_INFO("wait_status_busy, WRPERR : error : remote address 0x%x", stm32x_info->flash_base);
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & FLASH_ERROR) {
		/* If this operation fails, we ignore it and report the original
		* retval
		*/
		target_write_u32(target, stm32x_info->flash_base + FLASH_SR,
					status & FLASH_ERROR);
	}
	return retval;
}

static int stm32x_unlock_reg(struct flash_bank *bank)
{
	uint32_t ctrl;
	struct stm32l4x_flash_bank *stm32x_info = bank->driver_priv;
	struct target *target = bank->target;

	/* first check if not already unlocked
	 * otherwise writing on STM32_FLASH_KEYR will fail
	 */
	int retval = target_read_u32(target, stm32x_info->flash_base + FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & FLASH_LOCK) == 0)
		return ERROR_OK;

	/* unlock flash registers */
	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_KEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_KEYR, KEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, stm32x_info->flash_base + FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FLASH_LOCK) {
		LOG_ERROR("flash not unlocked STM32_FLASH_CR: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32x_unlock_option_reg(struct flash_bank *bank)
{
	uint32_t ctrl;
	struct stm32l4x_flash_bank *stm32x_info = bank->driver_priv;
	struct target *target = bank->target;

	int retval = target_read_u32(target, stm32x_info->flash_base + FLASH_OPTR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & OPT_LOCK) == 0)
		return ERROR_OK;

	/* unlock option registers */
	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_OPTKEYR, OPTKEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_OPTKEYR, OPTKEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, stm32x_info->flash_base + FLASH_OPTR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & OPT_LOCK) {
		LOG_ERROR("options not unlocked STM32_FLASH_OPTCR: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32x_read_options(struct flash_bank *bank)
{
	uint32_t optiondata;
	struct stm32l4x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;

	stm32x_info = bank->driver_priv;

	/* read current option bytes */
	int retval = target_read_u32(target, stm32x_info->flash_base + FLASH_OPTR, &optiondata);
	if (retval != ERROR_OK)
		return retval;

	stm32x_info->option_bytes.RDP = optiondata & 0xff;

	if (stm32x_info->option_bytes.RDP != 0xAA)
		LOG_INFO("Device Security Bit Set, current RDP 0x%x", stm32x_info->option_bytes.RDP);

	retval = target_read_u32(target, stm32x_info->flash_base + FLASH_WRP1AR + bank->bank_number * 0x20,
				 &optiondata);
	if (retval != ERROR_OK)
		return retval;

	stm32x_info->write_protected_a_start = (optiondata & 0xFF) * FLASH_SECTOR_SIZE;
	stm32x_info->write_protected_a_end = (optiondata>>16) * FLASH_SECTOR_SIZE;

	retval = target_read_u32(target, stm32x_info->flash_base + FLASH_WRP1BR + bank->bank_number * 0x20,
				 &optiondata);
	if (retval != ERROR_OK)
		return retval;

	stm32x_info->write_protected_b_start = (optiondata & 0xFF) * FLASH_SECTOR_SIZE;
	stm32x_info->write_protected_b_end = (optiondata>>16) * FLASH_SECTOR_SIZE;

	return ERROR_OK;
}

static int stm32x_write_options(struct flash_bank *bank)
{
	struct stm32l4x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;
	uint32_t optiondata;

	stm32x_info = bank->driver_priv;

	int retval = stm32x_unlock_option_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	/* rebuild option data */
	optiondata = stm32x_info->option_bytes.RDP;

	/* program options */
	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_OPTR, optiondata);
	if (retval != ERROR_OK)
		return retval;

	optiondata = (stm32x_info->write_protected_a_start/FLASH_SECTOR_SIZE) |
			(stm32x_info->write_protected_a_end/FLASH_SECTOR_SIZE)<<16;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_WRP1AR + bank->bank_number * 0x20,
					optiondata);
	if (retval != ERROR_OK)
		return retval;

	optiondata = (stm32x_info->write_protected_b_start/FLASH_SECTOR_SIZE) |
			(stm32x_info->write_protected_b_end/FLASH_SECTOR_SIZE)<<16;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_WRP1BR + bank->bank_number * 0x20,
					optiondata);
	if (retval != ERROR_OK)
		return retval;

	/* start programming cycle */
	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_OPTR, optiondata | OPT_START);
	if (retval != ERROR_OK)
		return retval;

	/* wait for completion */
	retval = stm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* relock registers */
	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_OPTR, optiondata | OPT_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_protect_check(struct flash_bank *bank)
{
	struct stm32l4x_flash_bank *stm32x_info = bank->driver_priv;

	/* read write protection settings */
	int retval = stm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	for (int i = 0; i < bank->num_sectors; i++) {
		if ((bank->sectors[i].offset >  stm32x_info->write_protected_a_start &&
			bank->sectors[i].offset <= stm32x_info->write_protected_a_end) ||
			(bank->sectors[i].offset >  stm32x_info->write_protected_b_start &&
			bank->sectors[i].offset <= stm32x_info->write_protected_b_end))
			bank->sectors[i].is_protected = 1;
	    else
			bank->sectors[i].is_protected = 0;
	}

	return ERROR_OK;
}

static int stm32x_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	struct stm32l4x_flash_bank *stm32x_flash = bank->driver_priv;
	const struct stm32l4x_part_info *part_info = stm32x_flash->part_info;
	unsigned int i;
	int retval;

	assert(first < bank->num_sectors);
	assert(last < bank->num_sectors);

	if (bank->target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = stm32x_unlock_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	/*
	Sector Erase
	To erase a sector, follow the procedure below:
	1. Check that no Flash memory operation is ongoing by checking the BSY bit in the
	  FLASH_SR register
	2. Set the PER bit and select the sector you wish to erase (SNB)
	   in the FLASH_CR register
	   if there is the second bank, set the FLASH_BKER bank erase bit
	3. Set the FLASH_START bit in the FLASH_CR register
	4. Wait for the BSY bit to be cleared
	*/

	for (i = first; i <= (unsigned int)last; i++) {
		if ( i < part_info->first_bank_size_kb )
		  	retval = target_write_u32(target, stm32x_flash->flash_base + FLASH_CR,
					FLASH_PER | FLASH_SNB(i) | FLASH_START);
		else
			retval = target_write_u32( target, stm32x_flash->flash_base + FLASH_CR,
					FLASH_BKER | FLASH_PER | FLASH_SNB((i + part_info->hole_sectors)) | FLASH_START);
		if (retval != ERROR_OK) {
			LOG_ERROR("erase sector error %d", i);
			return retval;
		}

		retval = stm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if ( retval != ERROR_OK) {
			LOG_ERROR("erase time-out error sector %d", i);
		return retval;
		}

		bank->sectors[i].is_erased = 1;
	}

	retval = target_write_u32(target, stm32x_flash->flash_base + FLASH_CR, FLASH_LOCK);
	if (retval != ERROR_OK) {
		LOG_ERROR("error during the lock of flash");
		return retval;
	}
	return ERROR_OK;
}

static int stm32x_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct target *target = bank->target;
	struct stm32l4x_flash_bank *stm32x_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_INFO("stm32x_protect, target_*_u32 : error : remote address 0x%x", stm32x_info->flash_base);

	/* read protection settings */
	int retval = stm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	for (int i = first; i <= last; i++) {

		if (set)
			bank->sectors[i].is_protected = 1;
		else
			bank->sectors[i].is_protected = 0;
	}


	/* analyse the sectors protected to create zone of wpr */

	uint32_t zone_a_start = 255;
	uint32_t zone_a_end = 255;

	uint32_t zone_b_start = 255;
	uint32_t zone_b_end = 255;

	stm32x_info->write_protected_a_start = zone_a_start;
	stm32x_info->write_protected_a_end = zone_a_end;

	stm32x_info->write_protected_b_start = zone_b_start;
	stm32x_info->write_protected_b_end = zone_b_end;

	retval = stm32x_write_options(bank);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_write_block(struct flash_bank *bank, const uint8_t *buffer,
			      uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size = BUFFER_SIZE;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[4];
	struct armv7m_algorithm armv7m_info;
	struct stm32l4x_flash_bank *stm32x_info = bank->driver_priv;

	int retval = ERROR_OK;

	/* see flash/smt32_flash_64.c for src */
	static const uint8_t stm32x_flash_write_code[] = {
		0x17, 0x4D,		/*		ldr	r5, .L11		*/
		0x2C, 0x68,		/*		ldr	r4, [r5]		*/
		0x44, 0xF0, 0x01, 0x04,	/*		orr	r4, r4, #1		*/
		0x2C, 0x60,		/*		str	r4, [r5]		*/
		0x00, 0x27,		/*		movs	r7, #0			*/
		0x08, 0x39,		/*		subs	r1, r1, #8		*/
		0x00, 0xF1, 0x08, 0x0C,	/*		add	ip, r0, #8		*/
		0x14, 0x4E,		/*		ldr	r6, .L11+4		*/
		0x1A, 0xE0,		/*		b	.L2			*/
					/*	.L4:					*/
		0x04, 0x68,		/*		ldr	r4, [r0]		*/
		0xD4, 0xB1,		/*		cbz	r4, .L3 		*/
					/*	.L10:					*/
		0x45, 0x68,		/*		ldr	r5, [r0, #4]		*/
		0x04, 0x68,		/*		ldr	r4, [r0]		*/
		0xA5, 0x42,		/*		cmp	r5, r4			*/
		0xF9, 0xD0,		/*		beq	.L4			*/
		0x02, 0xF1, 0x08, 0x0E,	/*		add	lr, r2, #8		*/
		0x44, 0x68,		/*		ldr	r4, [r0, #4]		*/
		0xD4, 0xE9, 0x00, 0x45,	/*		ldrd	r4, [r4]		*/
		0xC2, 0xE9, 0x00, 0x45,	/*		strd	r4, [r2]		*/
		0x42, 0x68,		/*		ldr	r2, [r0, #4]		*/
		0x8A, 0x42,		/*		cmp	r2, r1			*/
		0x2F, 0xBF,		/*		iteee	cs			*/
		0xC0, 0xF8, 0x04, 0xC0,	/*		strcs	ip, [r0, #4]		*/
		0x42, 0x68,		/*		ldrcc	r2, [r0, #4]		*/
		0x08, 0x32,		/*		addcc	r2, r2, #8		*/
		0x42, 0x60,		/*		strcc	r2, [r0, #4]		*/
					/*	.L7:					*/
		0x32, 0x68,		/*		ldr	r2, [r6]		*/
		0x12, 0xF4, 0x80, 0x3F,	/*		tst	r2, #65536		*/
		0xFB, 0xD1,		/*		bne	.L7			*/
		0x01, 0x37,		/*		adds	r7, r7, #1		*/
		0x72, 0x46,		/*		mov	r2, lr			*/
					/*	.L2:					*/
		0x9F, 0x42,		/*		cmp	r7, r3			*/
		0xE4, 0xD1,		/*		bne	.L10			*/
					/*	.L3:					*/
		0x03, 0x4A,		/*		ldr	r2, .L11		*/
		0x13, 0x68,		/*		ldr	r3, [r2]		*/
		0x23, 0xF0, 0x01, 0x03,	/*		bic	r3, r3, #1		*/
		0x13, 0x60,		/*		str	r3, [r2]		*/
		0x02, 0x4B,		/*		ldr	r3, .L11+4		*/
		0x18, 0x68,		/*		ldr	r0, [r3]		*/
					/*	.L12:					*/
		0x00, 0xBF,		/*		.align	2			*/
					/*	.L11:					*/
		0x14, 0x20, 0x02, 0x40,	/*		.word	1073881108		*/
		0x10, 0x20, 0x02, 0x40,	/*		.word	1073881104		*/
		0x00, 0xBE,		/* 		bkpt	#0x00			*/
	};

	if (target_alloc_working_area(target, sizeof(stm32x_flash_write_code),
				      &write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
				     sizeof(stm32x_flash_write_code),
				     stm32x_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);
			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);		/* buffer start, status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);		/* buffer end */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);		/* target address  */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);		/* count (word-64bit) */

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, address);
	buf_set_u32(reg_params[3].value, 0, 32, count);

	retval = target_run_flash_async_algorithm(target,
						  buffer,
						  count,
						  8,
						  0, NULL,
						  4, reg_params,
						  source->address,
						  source->size,
						  write_algorithm->address, 0,
						  &armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_INFO("error executing stm32l4x flash write algorithm");

		uint32_t error = buf_get_u32(reg_params[0].value, 0, 32) & FLASH_ERROR;

		if (error & FLASH_WRPERR)
			LOG_ERROR("flash memory write protected");

		if (error != 0) {
			LOG_ERROR("flash write failed = %08" PRIx32, error);
			/* Clear but report errors */
			target_write_u32(target, stm32x_info->flash_base + FLASH_SR, error);
			retval = ERROR_FAIL;
		}
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	return retval;
}

static int stm32x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	int retval;
	uint32_t count_written = count;

	struct stm32l4x_flash_bank *stm32x_info = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x8) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required 8-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	if ((count % 0x8) != 0) {
		LOG_WARNING("count 0x%" PRIx32 " breaks required 8-byte alignment", count);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}
	retval = stm32x_unlock_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	uint32_t words_remaining = (count_written / 8);
	/* multiple words (8-byte) to be programmed */
	if (words_remaining > 0) {
		/* try using a block write */
		retval = stm32x_write_block(bank, buffer, offset, words_remaining);
	  }

	if ((retval != ERROR_OK) && (retval != ERROR_TARGET_RESOURCE_NOT_AVAILABLE))
		return retval;

	return target_write_u32(target, stm32x_info->flash_base + FLASH_CR, FLASH_LOCK);
}

static int stm32x_read_id_code(struct flash_bank *bank, uint32_t *id)
{
	/* read stm32 device id register */
	int retval = target_read_u32(bank->target, DBGMCU_IDCODE_REGISTER, id);
	if (retval != ERROR_OK)
	  return retval;

	return ERROR_OK;
}

static int stm32x_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32l4x_flash_bank *stm32x_info = bank->driver_priv;
	uint32_t i;
	uint16_t flash_size_in_kb;
	uint32_t device_id;
	uint32_t base_address = FLASH_BANK0_ADDRESS;

	stm32x_info->probed = 0;

	int retval = stm32x_read_id_code(bank, &device_id);
	if (retval != ERROR_OK)
	  return retval;

	stm32x_info->idcode = device_id;

	LOG_DEBUG("Device id = 0x%08" PRIx32 "", device_id);

	for (unsigned int n = 0; n < ARRAY_SIZE(stm32l4x_parts); n++) {
	  if ((device_id & 0xfff) == stm32l4x_parts[n].id)
	    stm32x_info->part_info = &stm32l4x_parts[n];
	}

	if (!stm32x_info->part_info) {
	  LOG_WARNING("Cannot identify target as a STM32L4xx family.");
	  return ERROR_FAIL;
	}

	stm32x_info->flash_base = stm32x_info->part_info->flash_base;

	/* Get the flash size from target. */
	retval = target_read_u16(target, stm32x_info->part_info->fsize_base, &flash_size_in_kb);

#if 1
	/* Bug in the size returned by the L4 cut1. Take only 12 bits.*/
	flash_size_in_kb = flash_size_in_kb  & 0xFFF;
#endif
	flash_size_in_kb = stm32x_info->part_info->max_flash_size_kb;

	LOG_INFO("STM32L4xx flash size is %dkb, base address is 0x%" PRIx32, flash_size_in_kb, base_address);
	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (stm32x_info->user_bank_size) {
	  flash_size_in_kb = stm32x_info->user_bank_size / 1024;
	  LOG_INFO("ignoring flash probed value, using configured bank size: %dkbytes", flash_size_in_kb);
	}

	/* calculate numbers of sectors (2kB per sector) */
	uint32_t num_sectors =  (flash_size_in_kb * 1024) / FLASH_SECTOR_SIZE;

	if (bank->sectors) {
	  free(bank->sectors);
	  bank->sectors = NULL;
	}

	bank->size = flash_size_in_kb * 1024;
	bank->base = base_address;
	bank->num_sectors = num_sectors;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_sectors);
	if (bank->sectors == NULL) {
	  LOG_ERROR("failed to allocate bank sectors");
	  return ERROR_FAIL;
	}

	for (i = 0; i < num_sectors; i++) {
	  bank->sectors[i].offset = i * FLASH_SECTOR_SIZE;
	  bank->sectors[i].size = FLASH_SECTOR_SIZE;
	  bank->sectors[i].is_erased = -1;
	  bank->sectors[i].is_protected = 1;
	}

	stm32x_info->probed = 1;
	return ERROR_OK;
}

static int stm32x_auto_probe(struct flash_bank *bank)
{
	struct stm32l4x_flash_bank *stm32x_info = bank->driver_priv;

	if (stm32x_info->probed)
	  return ERROR_OK;

	return stm32x_probe(bank);
}

/* This method must return a string displaying information about the bank */
static int get_stm32x_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct stm32l4x_flash_bank *stm32x_info = bank->driver_priv;

	if (!stm32x_info->probed) {
	  int retval = stm32x_probe(bank);
	  if (retval != ERROR_OK) {
			snprintf(buf, buf_size, "Unable to find bank information.");
	    return retval;
	  }
	}
	const struct stm32l4x_part_info *info = stm32x_info->part_info;

	if (info) {
		const char *rev_str = NULL;
		uint16_t rev_id = stm32x_info->idcode >> 16;

		for (unsigned int i = 0; i < info->num_revs; i++) {
			if (rev_id == info->revs[i].rev)
				rev_str = info->revs[i].str;

			if (rev_str != NULL)
				snprintf(buf, buf_size, "%s - Rev: %s",
						stm32x_info->part_info->device_str, rev_str);
			else
				snprintf(buf, buf_size, "%s - Rev: unknown (0x%04x)",
						stm32x_info->part_info->device_str, rev_id);
		}
		return ERROR_OK;
	} else {
		snprintf(buf, buf_size, "Cannot identify target as a STM32L4x");
		return ERROR_FAIL;
	}
}

/* Static methods for mass erase stuff implementation */
#if 0
static int stm32x_unlock_options_bytes(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32l4x_flash_bank *stm32x_info = bank->driver_priv;
	int retval;
	uint32_t reg32;

	/*
	 * Unlocking the options bytes is done by unlocking the CR,
	 * then by writing the 2 FLASH_PEKEYR to the FLASH_OPTKEYR register
	 */

	/* check flash is not already unlocked */
	retval = target_read_u32(target, stm32x_info->flash_base + FLASH_CR,
				&reg32);
	if (retval != ERROR_OK)
		return retval;

	if ((reg32 & FLASH_OPTLOCK) == 0)
		return ERROR_OK;

	if ((reg32 & FLASH_LOCK) != 0) {
		retval = target_write_u32(target, stm32x_info->flash_base + FLASH_KEYR,
					KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_KEYR,
				    KEY2);
	if (retval != ERROR_OK)
	  	return retval;
	}
	  	
	/* To unlock the PECR write the 2 OPTKEY to the FLASH_OPTKEYR register */
	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_OPTKEYR,
			    OPTKEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_OPTKEYR,
			    OPTKEY2);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}
#endif

static int stm32x_mass_erase(struct flash_bank *bank)
{
	int retval;
	struct target *target = bank->target;
	struct stm32l4x_flash_bank *stm32x_info = NULL;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	stm32x_info = bank->driver_priv;

	retval = stm32x_unlock_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT/10);
	if (retval != ERROR_OK)
		return retval;

	uint32_t reg32;
	retval = target_read_u32(target, stm32x_info->flash_base + FLASH_CR, &reg32);
	if (retval != ERROR_OK)
		return retval;

	/* mass erase flash memory : two bank, two bit to set MER1, MER2*/
	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, reg32 | FLASH_MER2 | FLASH_MER1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, FLASH_MER2 | FLASH_MER1 | FLASH_START);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, stm32x_info->flash_base + FLASH_CR, &reg32);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, reg32 | FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("Target option locked");
	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_lock_command)
{
	struct target *target = NULL;
	struct stm32l4x_flash_bank *stm32l4x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32l4x_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32x_read_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "%s failed to read options",
			      bank->driver->name);
		return ERROR_OK;
	}

	/* set readout protection */
	stm32l4x_info->option_bytes.RDP = 0;

	if (stm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "%s failed to lock device",
			      bank->driver->name);
		return ERROR_OK;
	}

	command_print(CMD_CTX, "%s locked", bank->driver->name);

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_unlock_command)
{
	struct target *target = NULL;
	struct stm32l4x_flash_bank *stm32x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32x_read_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "%s failed to read options", bank->driver->name);
		return ERROR_OK;
	}

	/* clear readout protection and complementary option bytes
	 * this will also force a device unlock if set */
	stm32x_info->option_bytes.RDP = 0xAA;

	if (stm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "%s failed to unlock device", bank->driver->name);
		return ERROR_OK;
	}

	command_print(CMD_CTX, "%s unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.", bank->driver->name);

	return ERROR_OK;
}


COMMAND_HANDLER(stm32x_handle_mass_erase_command)
{
	int i;

	if (CMD_ARGC < 1) {
		command_print(CMD_CTX, "stm32l4x mass_erase <bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_mass_erase(bank);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD_CTX, "stm32l4x mass erase complete");
	} else {
		command_print(CMD_CTX, "stm32l4x mass erase failed");
	}

	return retval;
}


static const struct command_registration stm32x_exec_command_handlers[] = {
	{
		.name = "lock",
		.handler = stm32x_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.handler = stm32x_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Unlock entire protected flash device.",
	},

	{
		.name = "mass_erase",
		.handler = stm32x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "protect",
		/*		.handler = stm32x_protect,*/
		.mode = COMMAND_EXEC,
		.usage = "bank_id set/unset start_sector end_sector",
		.help = "write protect from start_sector to end_sector.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stm32x_command_handlers[] = {
	{
		.name = "stm32l4x",
		.mode = COMMAND_ANY,
		.help = "stm32l4x flash command group",
		.usage = "",
		.chain = stm32x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver stm32l4x_flash = {
	.name = "stm32l4x",
	.commands = stm32x_command_handlers,
	.flash_bank_command = stm32x_flash_bank_command,
	.erase = stm32x_erase,
	.protect = stm32x_protect,
	.write = stm32x_write,
	.read = default_flash_read,
	.probe = stm32x_probe,
	.auto_probe = stm32x_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = stm32x_protect_check,
	.info = get_stm32x_info,
};

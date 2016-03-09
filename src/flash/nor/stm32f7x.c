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

/*
 * To reduce testing complexity and dangers of regressions,
 * a separate file is used for stm32F7xx series. The sector size 
 * is twice larger than F2/F4
 */

/* Erase time can be as high as 1000ms, 10x this and it's toast... */
#define FLASH_ERASE_TIMEOUT 10000
#define FLASH_WRITE_TIMEOUT 5

/* RM 0385 & RM 0410 */
#define FLASH_KEYR	0x04
#define FLASH_OPTKEYR	0x08
#define FLASH_SR	0x0C
#define FLASH_CR	0x10
#define FLASH_OPTR      0x14

/* FLASH_CR register bits */
#define FLASH_PG       (1 << 0)
#define FLASH_SER      (1 << 1)
#define FLASH_MER      (1 << 2)
#define FLASH_MER2     (1 << 15)
#define FLASH_START    (1 << 16)
#define FLASH_LOCK     (1 << 31)

#define FLASH_PSIZE_8  (0 << 8)
#define FLASH_PSIZE_32 (2 << 8)

#define FLASH_SNB(a)   ((((a) >= 12) ? 0x10 | ((a) - 12) : (a)) << 3)

/* FLASH_SR register bits */
#define FLASH_BSY      (1 << 16) /* Operation in progress */
#define FLASH_PGSERR   (1 << 7) /* Programming sequence error */
#define FLASH_PGPERR   (1 << 6) /* Programming parallelism error */
#define FLASH_PGAERR   (1 << 5) /* Programming alignment error */
#define FLASH_WRPERR   (1 << 4) /* Write protection error */
#define FLASH_PROGERR  (1 << 3)  /* Write protection error */
#define FLASH_OPERR    (1 << 1) /* Operation error */

#define FLASH_ERROR (FLASH_PROGERR | FLASH_PGSERR | FLASH_PGPERR | FLASH_PGAERR | FLASH_WRPERR | FLASH_OPERR)

/* STM32_FLASH_OPTR register bits */
#define OPT_LOCK      (1 << 0)
#define OPT_START     (1 << 1)

/* STM32_FLASH_OBR bit definitions (reading) */
#define OPT_ERROR      0
#define OPT_READOUT    1
#define OPT_RDWDGSW    2
#define OPT_RDRSTSTOP  3
#define OPT_RDRSTSTDBY 4
#define OPT_RDDUALBANK 5

/* register unlock keys */
#define KEY1           0x45670123
#define KEY2           0xCDEF89AB

/* option register unlock key */
#define OPTKEY1        0x08192A3B
#define OPTKEY2        0x4C5D6E7F

/* option bytes */
#define OPTION_BYTES_ADDRESS 0x1FFC000

#define IDWG_SW    (1 << 5)
#define WWWG_SW    (1 << 4)

#define DBGMCU_IDCODE_REGISTER 0xE0042000
#define FLASH_BANK0_ADDRESS 0x08000000
#define FLASH_BASE 0x40023C00


struct stm32f7x_rev {
	uint16_t rev;
	const char *str;
};

struct stm32x_options {
	uint8_t RDP;
	uint32_t protection;
	uint8_t user_options;
	uint8_t user2_options;
	uint8_t window_watchdog_selection;
	uint8_t independent_watchdog_selection;
};

struct stm32f7x_part_info {
	uint16_t id;
	const char *device_str;
	const struct stm32f7x_rev *revs;
	size_t num_revs;
	unsigned int page_size;
	unsigned int pages_per_sector;
	uint16_t max_flash_size_kb;
	uint8_t has_dual_bank;
	uint16_t user_bank_size;
	uint32_t flash_base;	/* Flash controller registers location */
	uint32_t fsize_base;	/* Location of FSIZE register */
};

struct stm32f7x_flash_bank {
	int probed;
	uint32_t idcode;
	uint32_t user_bank_size;
	uint32_t flash_base;    /* Address of flash memory */
	struct stm32x_options option_bytes;
	const struct stm32f7x_part_info *part_info;
};

static const struct stm32f7x_rev stm32_449_revs[] = {
	{ 0x1000, "A" }, { 0x1001, "Z" },
};

static const struct stm32f7x_rev stm32_451_revs[] = {
	{ 0x1000, "A" },
};


static const struct stm32f7x_part_info stm32f7x_parts[] = {
	{
	  .id				= 0x449,
	  .revs				= stm32_449_revs,
	  .num_revs			= ARRAY_SIZE(stm32_449_revs),
		.device_str			= "STM32F74/F75xx 1M",
	  .page_size			= 256,  /* 256 KB */
	  .max_flash_size_kb		= 1024,
		.has_dual_bank		= 0,
		.flash_base			= 0x40023C00,
		.fsize_base			= 0x1FF0F442,
	},
	{
		.id					= 0x451,
		.revs				= stm32_451_revs,
		.num_revs			= ARRAY_SIZE(stm32_451_revs),
		.device_str			= "STM32F76/F77xx 2M",
		.page_size			= 256,  /* 256 KB */
		.max_flash_size_kb	= 2048,
		.has_dual_bank		= 1,
	  .flash_base			= 0x40023C00,
	  .fsize_base			= 0x1FF0F442,
	},
};

static int stm32x_unlock_reg(struct flash_bank *bank);
static int stm32x_probe(struct flash_bank *bank);

/* flash bank stm32x <base> <size> 0 0 <target#> */

FLASH_BANK_COMMAND_HANDLER(stm32x_flash_bank_command)
{
	struct stm32f7x_flash_bank *stm32x_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	stm32x_info = malloc(sizeof(struct stm32f7x_flash_bank));
	bank->driver_priv = stm32x_info;

	stm32x_info->probed = 0;
	stm32x_info->user_bank_size = bank->size;
	stm32x_info->flash_base = FLASH_BASE;

	return ERROR_OK;
}

static inline int stm32x_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	struct stm32f7x_flash_bank *stm32x_info = bank->driver_priv;

	return target_read_u32(target, stm32x_info->flash_base + FLASH_SR, status);
}

static int stm32x_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	struct stm32f7x_part_info *stm32x_info = bank->driver_priv;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = stm32x_get_flash_status(bank, &status);
		if (retval != ERROR_OK) {
			LOG_INFO("wait_status_busy, target_read_u32 : error : remote address 0x%x", stm32x_info->flash_base);
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
	struct stm32f7x_flash_bank *stm32x_info = bank->driver_priv;
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
	struct stm32f7x_flash_bank *stm32x_info = bank->driver_priv;
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
	struct stm32f7x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;

	stm32x_info = bank->driver_priv;

	/* read current option bytes */
	int retval = target_read_u32(target, stm32x_info->flash_base + FLASH_OPTR, &optiondata);
	if (retval != ERROR_OK)
		return retval;

	/* decode option data */
	stm32x_info->option_bytes.user_options = optiondata & 0xfc;
	stm32x_info->option_bytes.RDP = (optiondata >> 8) & 0xff;
	stm32x_info->option_bytes.protection = (optiondata >> 16) & 0xfff;
	stm32x_info->option_bytes.user2_options = (optiondata >> 24) & 0xf0;

	if (optiondata & IDWG_SW)
		stm32x_info->option_bytes.independent_watchdog_selection = 1;
	else
		stm32x_info->option_bytes.independent_watchdog_selection = 0;

	if (optiondata & WWWG_SW)
		stm32x_info->option_bytes.window_watchdog_selection = 1;
	else
		stm32x_info->option_bytes.window_watchdog_selection = 0;

	if (stm32x_info->option_bytes.RDP != 0xAA)
		LOG_INFO("Device Security Bit Set");

	return ERROR_OK;
}

static int stm32x_write_options(struct flash_bank *bank)
{
	struct stm32f7x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;
	uint32_t optiondata;

	stm32x_info = bank->driver_priv;

	int retval = stm32x_unlock_option_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	/* rebuild option data */
	optiondata = stm32x_info->option_bytes.user_options;
	optiondata |= (stm32x_info->option_bytes.RDP << 8);
	optiondata |= (stm32x_info->option_bytes.protection & 0xfff) << 16;
	optiondata |= (stm32x_info->option_bytes.user2_options & 0xf0) << 24;

	if (stm32x_info->option_bytes.independent_watchdog_selection)
		optiondata |= IDWG_SW;
	else
		optiondata &= ~IDWG_SW;

	if (stm32x_info->option_bytes.window_watchdog_selection)
		optiondata |= WWWG_SW;
	else
		optiondata &= ~WWWG_SW;

	/* program options */
	 retval = target_write_u32(target, stm32x_info->flash_base + FLASH_OPTR, optiondata);
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
	struct stm32f7x_flash_bank *stm32x_info = bank->driver_priv;

	/* read write protection settings */
	int retval = stm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	for (int i = 0; i < bank->num_sectors; i++) {
		if (stm32x_info->part_info->has_dual_bank &&
			((stm32x_info->option_bytes.user2_options & (1 << OPT_RDDUALBANK)) == 0)) {
			/* double bank */
			if (stm32x_info->option_bytes.protection & (1 << (i/2)))
				bank->sectors[i].is_protected = 0;
			else
				bank->sectors[i].is_protected = 1;
		} else {
		if (stm32x_info->option_bytes.protection & (1 << i))
			bank->sectors[i].is_protected = 0;
		else
			bank->sectors[i].is_protected = 1;
	}
	}

	return ERROR_OK;
}

static int stm32x_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	struct stm32f7x_flash_bank *stm32x_info = bank->driver_priv;
	int i;
	int retval;
	int offset;

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
	2. Set the SER bit and select the sector
	  you wish to erase (SNB) in the FLASH_CR register
	3. Set the STRT bit in the FLASH_CR register
	4. Wait for the BSY bit to be cleared
	 */
	for (i = first; i <= last; i++) {
		offset = 0;
		if (stm32x_info->part_info->has_dual_bank && \
		  ((stm32x_info->option_bytes.user2_options & (1 << OPT_RDDUALBANK)) == 0) && \
			(bank->size/1024) < stm32x_info->part_info->max_flash_size_kb && \
			i >= (bank->num_sectors/2)) {
			/* Dual bank: Hole in sector nb: Bank2 start on sector 12 */
			offset = 12 - (bank->num_sectors/2);
			i += offset;
			LOG_DEBUG("Dual Bank on %dKb part: erase sector %d", bank->size/1024, i);
		}
		else
		LOG_DEBUG("erase sector %d", i);
		retval = target_write_u32(target,
					stm32x_info->flash_base + FLASH_CR, 
					FLASH_SER | FLASH_SNB(i) | FLASH_START | FLASH_PSIZE_32);
		if (retval != ERROR_OK) {
			LOG_ERROR("erase sector error %d", i);
			return retval;
		}

		retval = stm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK) {
			LOG_ERROR("erase time-out error sector %d", i);
			return retval;
		}
		if (offset)
			i -= offset;

		bank->sectors[i].is_erased = 1;
	}

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, FLASH_LOCK);
	if (retval != ERROR_OK) {
		LOG_ERROR("error during the lock of flash");
		return retval;
	}

	return ERROR_OK;
}

static int stm32x_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct target *target = bank->target;
	struct stm32f7x_flash_bank *stm32x_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* read protection settings */
	int retval = stm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	if (stm32x_info->part_info->has_dual_bank &&
		((stm32x_info->option_bytes.user2_options & (1 << OPT_RDDUALBANK))) == 0) {
		for (int i = first; i <= last; i++) {
			if (set)
				stm32x_info->option_bytes.protection &= ~(1 << (i/2));
			else {
				LOG_INFO("stm32x_protect : double bank reset %d, %d", i , i/2 );
				stm32x_info->option_bytes.protection |= (1 << (i/2));
			}
		}
	} else {
		LOG_INFO("stm32x_protect : mono bank ");
	for (int i = first; i <= last; i++) {
		if (set)
			stm32x_info->option_bytes.protection &= ~(1 << i);
		else
			stm32x_info->option_bytes.protection |= (1 << i);
	}
	}

	LOG_INFO("stm32x_protect, option_bytes written 0x%x", stm32x_info->option_bytes.protection);
	retval = stm32x_write_options(bank);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[4];
	struct armv7m_algorithm armv7m_info;
	struct stm32f7x_flash_bank *stm32x_info = bank->driver_priv;

	int retval = ERROR_OK;

	/* see flash/smt32_flash_32.c for src */
	static const uint8_t stm32x_flash_write_code[] = {
		0x43, 0xF6, 0x10, 0x44,		/*		movw	r4, #15376		*/
		0xC4, 0xF2, 0x02, 0x04,		/*		movt	r4, 16386		*/
		0x25, 0x68,			/*		ldr	r5, [r4, #0]		*/
		0x45, 0xF4, 0x00, 0x76,		/*		orr	r6, r5, #512		*/
		0x46, 0xF0, 0x01, 0x05,		/*		orr	r5, r6, #1		*/
		0x25, 0x60,			/*		str	r5, [r4, #0]		*/
		0x4F, 0xF0, 0x00, 0x07,		/*		mov	r7, #0			*/
		0xA1, 0xF1, 0x04, 0x01,		/*		sub	r1, r1, #4		*/
		0x00, 0xF1, 0x08, 0x0C,		/*		add	ip, r0, #8		*/
		0x43, 0xF6, 0x0C, 0x46,		/*		movw	r6, #15372		*/
		0xC4, 0xF2, 0x02, 0x06,		/*		movt	r6, 16386		*/
		0x25, 0xE0,			/*		b	.L2			*/
						/*	.L5:					*/
		0x04, 0x68,			/*		ldr	r4, [r0, #0]		*/
		0x1C, 0xB9,			/*		cbnz	r4, .L10		*/
		0x4F, 0xF0, 0x00, 0x03,		/*		mov	r3, #0			*/
		0x43, 0x60,			/*		str	r3, [r0, #4]		*/
		0x21, 0xE0,			/*		b	.L4			*/
						/*	.L10:					*/
		0x45, 0x68,			/*		ldr	r5, [r0, #4]		*/
		0x04, 0x68,			/*		ldr	r4, [r0, #0]		*/
		0xA5, 0x42,			/*		cmp	r5, r4			*/
		0xF5, 0xD0,			/*		beq	.L5			*/
		0x44, 0x68,			/*		ldr	r4, [r0, #4]		*/
		0x24, 0x68,			/*		ldr	r4, [r4, #0]		*/
		0x42, 0xF8, 0x04, 0x4B,		/*		str	r4, [r2], #4		*/
		0x44, 0x68,			/*		ldr	r4, [r0, #4]		*/
		0x8C, 0x42,			/*		cmp	r4, r1			*/
		0x2F, 0xBF,			/*		iteee	cs			*/
		0xC0, 0xF8, 0x04, 0xC0,		/*		strcs	ip, [r0, #4]		*/
		0x44, 0x68,			/*		ldrcc	r4, [r0, #4]		*/
		0x04, 0x34,			/*		addcc	r4, r4, #4		*/
		0x44, 0x60,			/*		strcc	r4, [r0, #4]		*/
		0xBF, 0xF3, 0x4F, 0x8F,		/*		dsb				*/
						/*	.L8:					*/
		0x34, 0x68,			/*		ldr	r4, [r6, #0]		*/
		0x14, 0xF4, 0x80, 0x3F,		/*		tst	r4, #65536		*/
		0xFB, 0xD1,			/*		bne	.L8			*/
		0x34, 0x68,			/*		ldr	r4, [r6, #0]		*/
		0x14, 0xF0, 0x01, 0x0F,		/*		tst	r4, #1			*/
		0x03, 0xD0,			/*		beq	.L9			*/
		0x34, 0x68,			/*		ldr	r4, [r6, #0]		*/
		0x44, 0xF0, 0x01, 0x04,		/*		orr	r4, r4, #1		*/
		0x34, 0x60,			/*		str	r4, [r6, #0]		*/
						/*	.L9:					*/
		0x07, 0xF1, 0x01, 0x07,		/*		add	r7, r7, #1		*/
						/*	.L2:					*/
		0x9F, 0x42,			/*		cmp	r7, r3			*/
		0xDD, 0xD1,			/*		bne	.L10			*/
						/*	.L4:					*/
		0x43, 0xF6, 0x0C, 0x40,		/*		movw	r0, #15372		*/
		0xC4, 0xF2, 0x02, 0x00,		/*		movt	r0, 16386		*/
		0x00, 0x68,			/*		ldr	r0, [r0, #0]		*/
		0x00, 0xBE,			/*		bkpt	#0x00			*/
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

	LOG_DEBUG("target_alloc_working_area_try : buffer_size -> 0x%x", buffer_size);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);		/* buffer start, status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);		/* buffer end */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);		/* target address  */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);		/* count (word-32bit) */

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, address);
	buf_set_u32(reg_params[3].value, 0, 32, count);

	retval = target_run_flash_async_algorithm(target,
						  buffer,
						  count,
						  4,
						  0, NULL,
						  4, reg_params,
						  source->address, source->size,
						  write_algorithm->address, 0,
						  &armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_INFO("error executing stm32f7x flash write algorithm");

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
	struct stm32f7x_flash_bank *stm32x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t address = bank->base + offset;
	uint32_t bytes_written = 0;
	uint32_t count_written = count;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32x_unlock_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	if (count < 4) {
		/* small transfer */
		retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, FLASH_PG | FLASH_PSIZE_8);
		if (retval != ERROR_OK)
			return retval;

		for (unsigned int i = 0; i < (4 - (address & 0x3)); i++) {
			retval = target_write_u8(target, address + i, buffer[i]);

		if (retval != ERROR_OK)
			return retval;

		offset++;
		count_written--;
		bytes_written++;
	  }
	}

	uint32_t words_remaining = (count_written / 4);
	uint32_t bytes_remaining = (count_written & 0x00000003);

	/* multiple half words (4-byte) to be programmed? */
	if (words_remaining > 0) {
		/* try using a block write */
		retval = stm32x_write_block(bank, buffer, offset, words_remaining);
		if (retval != ERROR_OK) {
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
				/* if block write failed (no sufficient working area),
				 * we use normal (slow) single dword accesses */
				LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
			}
		} else {
			buffer += words_remaining * 4;
			address += words_remaining * 4 + bytes_written;
			words_remaining = 0;
		}
	}

	if ((retval != ERROR_OK) && (retval != ERROR_TARGET_RESOURCE_NOT_AVAILABLE))
		return retval;

	/*
	Standard programming
	The Flash memory programming sequence is as follows:
	1. Check that no main Flash memory operation is ongoing by checking the BSY bit in the
	  FLASH_SR register.
	2. Set the PG bit in the FLASH_CR register
	3. Byte access 
	Wait for the BSY bit to be cleared
	*/
	if (bytes_remaining) {
		retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, FLASH_PG | FLASH_PSIZE_8);
		if (retval != ERROR_OK)
			return retval;

		LOG_DEBUG("bytes_remaining %d", bytes_remaining);
		for (unsigned int i = 0; i < bytes_remaining; i++) {
			retval = target_write_u8(target, address+i, buffer[bytes_written+i-2]);
			if (retval != ERROR_OK)
				return retval;
		}

		retval = stm32x_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;
	}

	return target_write_u32(target, stm32x_info->flash_base + FLASH_CR, FLASH_LOCK);
}

static void setup_sector(struct flash_bank *bank, int start, int num, int size)
{
	for (int i = start; i < (start + num) ; i++) {
		assert(i < bank->num_sectors);
		bank->sectors[i].offset = bank->size;
		bank->sectors[i].size = size;
		bank->size += bank->sectors[i].size;
	}
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
	struct stm32f7x_flash_bank *stm32x_info = bank->driver_priv;
	int i;
	uint16_t flash_size_in_kb;
	uint32_t device_id;
	uint32_t base_address = FLASH_BANK0_ADDRESS;

	stm32x_info->probed = 0;

	int retval = stm32x_read_id_code(bank, &stm32x_info->idcode);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("device id = 0x%08" PRIx32 "", stm32x_info->idcode);

	device_id = stm32x_info->idcode & 0xfff;

	for (unsigned int n = 0; n < ARRAY_SIZE(stm32f7x_parts); n++) {
		if ((device_id & 0xfff) == stm32f7x_parts[n].id)
			stm32x_info->part_info = &stm32f7x_parts[n];
	}
	if (!stm32x_info->part_info) {
		LOG_WARNING("Cannot identify target as a STM32F7xx family.");
		return ERROR_FAIL;
	}

	/* update the address of controller from data base */
	stm32x_info->flash_base = stm32x_info->part_info->flash_base;

	/* read the option bytes (dual bank) */
	retval = stm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	/* if device has Read Protection level 1, flash size register read will fail */
	if (stm32x_info->option_bytes.RDP != 0xAA)
		/* force the value to get the max flash size */
		flash_size_in_kb = 0xffff;
	else {
	/* get flash size from target. */
	retval = target_read_u16(target, stm32x_info->part_info->fsize_base, &flash_size_in_kb);
	LOG_INFO("flash size probed value %d", flash_size_in_kb);
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (stm32x_info->user_bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = stm32x_info->user_bank_size / 1024;
	} else if (flash_size_in_kb == 0xffff) {
		/* die flash size */
		flash_size_in_kb = stm32x_info->part_info->max_flash_size_kb;
	}

	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xffff);

	/* calculate numbers of pages */
	int num_pages;

	if (stm32x_info->part_info->has_dual_bank &&
			((stm32x_info->option_bytes.user2_options & (1 << OPT_RDDUALBANK)) == 0))
		num_pages = ((flash_size_in_kb / 256) + 4) * 2;
	else
		num_pages = (flash_size_in_kb / 256) + 4;

	/* check that calculation result makes sense */
	assert(num_pages > 0);

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->base = base_address;
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
	bank->size = 0;

	if (stm32x_info->part_info->has_dual_bank && \
			((stm32x_info->option_bytes.user2_options & (1 << OPT_RDDUALBANK)) == 0)) {
		if (flash_size_in_kb < stm32x_info->part_info->max_flash_size_kb) {
			/* dual bank but lower flash size means a hole between the 2 banks */
			/* fixed memory */
			setup_sector(bank, 0, 4, 16 * 1024);
			setup_sector(bank, 4, 1, 64 * 1024);

			/* dynamic memory */
			setup_sector(bank, 5, (num_pages/2) - 5, 128 * 1024);

			/* fixed memory */
			setup_sector(bank, (num_pages/2), 4, 16 * 1024);
			setup_sector(bank, (num_pages/2)+4, 1, 64 * 1024);

			/* dynamic memory */
			setup_sector(bank, (num_pages/2)+5, (num_pages/2) - 5, 128 * 1024);
		}
		else {
			/* dual bank organisation */
			/* fixed memory */
			setup_sector(bank, 0, 4, 16 * 1024);
			setup_sector(bank, 4, 1, 64 * 1024);

			/* dynamic memory */
			setup_sector(bank, 5, MIN(12, num_pages) - 5, 128 * 1024);

			/* fixed memory */
			setup_sector(bank, 12, 4, 16 * 1024);
			setup_sector(bank, 16, 1, 64 * 1024);

			/* dynamic memory */
			setup_sector(bank, 17, num_pages - 5 - 12, 128 * 1024);
		}
	} else {
	/* fixed memory */
	setup_sector(bank, 0, 4, 32 * 1024);
	setup_sector(bank, 4, 1, 128 * 1024);

	/* dynamic memory */
		setup_sector(bank, 5, num_pages - 5, 256 * 1024);
	}

	for (i = 0; i < num_pages; i++) {
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	stm32x_info->probed = 1;
	return ERROR_OK;
}

static int stm32x_auto_probe(struct flash_bank *bank)
{
	struct stm32f7x_flash_bank *stm32x_info = bank->driver_priv;

	if (stm32x_info->probed)
		return ERROR_OK;
	return stm32x_probe(bank);
}

/* This method must return a string displaying information about the bank */
static int stm32x_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct stm32f7x_flash_bank *stm32x_info = bank->driver_priv;
	const struct stm32f7x_part_info *info = stm32x_info->part_info;

	if (!stm32x_info->probed) {
		int retval = stm32x_probe(bank);
		if (retval != ERROR_OK) {
			snprintf(buf, buf_size, "Unable to find bank information.");
			return retval;
		}
	}

	if (info) {
		const char *rev_str = NULL;
		uint16_t rev_id = stm32x_info->idcode >> 16;

		for (unsigned int i = 0; i < info->num_revs; i++)
			if (rev_id == info->revs[i].rev)
				rev_str = info->revs[i].str;

		if (rev_str != NULL) {
			snprintf(buf, buf_size, "%s - Rev: %s",
				stm32x_info->part_info->device_str, rev_str);
		} else {
			snprintf(buf, buf_size,
				 "%s - Rev: unknown (0x%04x)",
				stm32x_info->part_info->device_str, rev_id);
		}

		return ERROR_OK;
	} else {
	  snprintf(buf, buf_size, "Cannot identify target as a STM32F7x");
	  return ERROR_FAIL;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_lock_command)
{
	struct target *target = NULL;
	struct stm32f7x_flash_bank *stm32f7x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32f7x_info = bank->driver_priv;
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
	stm32f7x_info->option_bytes.RDP = 0;

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
	struct stm32f7x_flash_bank *stm32x_info = NULL;

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

static int stm32x_mass_erase(struct flash_bank *bank)
{
	int retval;
	uint32_t FLASH_MER_CMD = FLASH_MER;
	struct target *target = bank->target;
	struct stm32f7x_flash_bank *stm32x_info = NULL;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	stm32x_info = bank->driver_priv;

	retval = stm32x_unlock_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	if (stm32x_info->part_info->has_dual_bank)
		FLASH_MER_CMD |= FLASH_MER2;

	/* mass erase flash memory */
	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, 
								FLASH_MER_CMD);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR,
								FLASH_MER_CMD | FLASH_START);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32x_wait_status_busy(bank, 30000);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_mass_erase_command)
{
	int i;

	if (CMD_ARGC < 1) {
		command_print(CMD_CTX, "stm32f7x mass_erase <bank>");
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

		command_print(CMD_CTX, "stm32f7x mass erase complete");
	} else {
		command_print(CMD_CTX, "stm32f7x mass erase failed");
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
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stm32x_command_handlers[] = {
	{
		.name = "stm32f7x",
		.mode = COMMAND_ANY,
		.help = "stm32f7x flash command group",
		.usage = "",
		.chain = stm32x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver stm32f7x_flash = {
	.name = "stm32f7x",
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
	.info = stm32x_get_info,
};

#!/usr/bin/perl
#
# Helper for generating GPIO setup for STM32F0, F4, F7, H7, L0, L1, L4, L4+
# (for 'stmqspi' and 'msoftspi' drivers).
#
# Each pin is configured by "PortAndBit:Conf:Speed"
#  'PortAndBit' specifies Port and bit number
#  'Conf' is one of 'PP' (push-pull output), 'PO' (open drain output), 'IN' (input),
#     optionally followed by 'UP' (pull-up), or 'DO' (pull-down), or 'AFXX' (alternate function)
#  'Speed' is one of 'L' (low), 'M' (medium), 'H' (high), 'V' (very high)
#
# Port configuration can be given on command line as a single string (pins separated by commas)
# or via CubeMX generated file. The latter must consist of the quadspi.c / octospi.c and the
# corresponding header. The precise spelling in these files doesn't seem to be consistent, though ...
#
use strict;
use Getopt::Std;

my $GPIO_BASE;
my $Conf;

# mini-stm32f030f4p6
#$GPIO_BASE = 0x48000000;
#$Conf = "PA07:PP:V, PA06:INUP:V, PA05:PP:V, PA04:PP:L, PB01:PP:V";

# stm32f412g-disco
#$GPIO_BASE = 0x40020000;
#$Conf = "PB02:AF09:V, PF09:AF10:V, PF08:AF10:V, PF07:AF09:V, PF06:AF09:V, PG06:AF10:V";

# stm32f469i-disco
#$GPIO_BASE = 0x40020000;
#$Conf = "PB06:AF10:V, PF10:AF09:V, PF09:AF10:V, PF08:AF10:V, PF07:AF09:V, PF06:AF09:V";

# stm32f723e-dicso
#$GPIO_BASE = 0x40020000;
#$Conf = "PB06:AF10:V, PB02:AF09:V, PC10:AF09:V, PC09:AF09:V, PD13:AF09:V, PE02:AF09:V";

# stm32f746g-disco
#$GPIO_BASE = 0x40020000;
#Conf = "PB06:AF10:V, PB02:AF09:V, PD13:AF09:V, PD12:AF09:V, PD11:AF09:V, PE02:AF09:V";

# stm32f769i-disco
#$GPIO_BASE = 0x40020000;
#$Conf = "PB06:AF10:V, PB02:AF09:V, PC10:AF09:V, PC09:AF09:V, PD13:AF09:V, PE02:AF09:V";

# stm32l476g-disco
#$GPIO_BASE = 0x48000000;
#$Conf = "PE15:AF10:V, PE14:AF10:V, PE13:AF10:V, PE12:AF10:V, PE11:AF10:V, PE10:AF10:V";

# stm32l496g-disco
#$GPIO_BASE = 0x48000000;
#$Conf = "PA07:AF10:V, PA06:AF10:V, PA03:AF10:V, PB11:AF10:V, PB01:AF10:V, PB00:AF10:V";

# stm32l4r9i-disco
#$GPIO_BASE = 0x48000000;
#$Conf = "PG15:AF05:V, PG12:AF05:V, PG10:AF05:V, PG09:AF05:V, PH10:AF05:V, PH09:AF05:V, "
#      . "PH08:AF05:V, PI11:AF05:V, PI10:AF05:V, PI09:AF05:V, PI06:AF05:V";

# nucleo-f767zi dual
#$GPIO_BASE = 0x40020000;
#$Conf = "PB06:AF10:V, PB02:AF09:V, PC11:AF09:V, PD13:AF09:V, PD12:AF09:V, PD11:AF09:V, "
#      . "PE07:AF10:V, PE02:AF09:V, PE08:AF10:V, PG14:AF09:V, PG09:AF09:V";

# nucleo-h743zi dual
#$GPIO_BASE = 0x58020000;
#$Conf = "PB06:AF10:V, PB02:AF09:V, PC11:AF09:V, PD13:AF09:V, PD12:AF09:V, PD11:AF09:V, "
#      . "PE07:AF10:V, PE02:AF09:V, PE08:AF10:V, PG14:AF09:V, PG09:AF09:V";

# nucleo-l4r5zi one dual-quad
#$GPIO_BASE = 0x48000000;
#$Conf = "PA02:AF10:V, PC04:AF10:V, PC03:AF10:V, PC02:AF10:V, PC01:AF10:V, PE15:AF10:V, "
#      . "PE14:AF10:V, PE13:AF10:V, PE12:AF10:V, PE10:AF10:V";

&getopts('b:c:f:');
if ($Getopt::Std::opt_b eq '')
{
	if ($GPIO_BASE eq '')
	{
		die("usage: $0 -b io_base [ -c port_configuration ] [ -f conf_file ]");
	}
}
else
{
	$GPIO_BASE = eval $Getopt::Std::opt_b;
}

if ($Getopt::Std::opt_c eq '')
{
	if (($Conf eq '') && ($Getopt::Std::opt_f eq ''))
	{
		die("usage: $0 [ -b io_base ] ( -c port_configuration | -f conf_file )");
	}
}
else
{
	$Conf = $Getopt::Std::opt_c . ',';
}

my $Sep = "\t";
my $Form = "${Sep}mmw 0x%08X 0x%08X 0x%08X\t;# ";

# these offsets are identical on all F0, F4, F7, H7, L4, L4+ devices up to now
my $GPIO_OFFS = 0x400;
my $GPIO_MODER = 0x00;
my $GPIO_OTYPER = 0x04;
my $GPIO_OSPEEDR = 0x08;
my $GPIO_PUPDR = 0x0C;
my $GPIO_IDR = 0x10;
my $GPIO_ODR = 0x14;
my $GPIO_BSRR = 0x18;
my $GPIO_LCKR = 0x1C;
my $GPIO_AFRL = 0x20;
my $GPIO_AFRH = 0x24;

my @Out = ( { }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { } );
my @Port = ( );
my %Conf;

my $pins;
my $altn;
my %defs;

if ($Getopt::Std::opt_f ne '')
{
	open(CONF_FILE, '<', $Getopt::Std::opt_f) || die("can't open $Getopt::Std::opt_f");
	while (my $line = <CONF_FILE>)
	{
		if ($line =~ /^\s*#define\s+(QSPI|OCTOSPI[^_]*)\w+_Pin/)
		{
			if ($line =~ /#define\s+(\w+)\s+(\w+)/)
			{
				$defs{$1} = $2;
			}
			else
			{
				die($line);
			}
		}
		elsif ($line =~ /^\s*(P[A-Z])([0-9]+)\s*-+>\s+(QUADSPI|OCTOSPI[^_]*)_(\w+)/)
		{
			$Conf{$4} = sprintf("%s%02d", $1, $2);
		}
		elsif ($line =~ /^\s*GPIO_InitStruct.Pin\s*=\s*([^;]+\w)/)
		{
			$pins = $1;
			while ($line !~ /;/)
			{
				$line = <CONF_FILE>;
				$line =~ /^\s*([^;]+\w)/;
				$pins .= $1;
			}
		}
		elsif ($line =~ /^\s*GPIO_InitStruct.Alternate\s*=\s*GPIO_AF([0-9]+)_(QUADSPI|OCTOSPI[^_]*)/)
		{
			$altn = $1;
		}
		elsif ($line =~ /^\s*HAL_GPIO_Init\s*\(GPIO([A-Z])\s*,/)
		{
			my $port = $1;
			my @pin = split(/\s*\|\s*/, $pins);
			foreach my $pin (@pin)
			{
				my $bit;
				if (exists($defs{$pin}))
				{
					$defs{$pin} =~ /GPIO_PIN_([0-9]+)/;
					$bit = $1;
				}
				else
				{
					$pin =~ /GPIO_PIN_([0-9]+)/;
					$bit = $1;
				}
				$Conf .= sprintf("P%s%02d:AF%02d:V, ", $port, $bit, $altn);
			}
			$pins = '';
			$altn = 0;
		}
	}
	close(CONF_FILE);
}

if (exists $Conf{'BK1_IO0'})
{
  /* QuadSPI on F4, F7, H7 */
	my $line;
	for my $i ('NCS', 'BK1_NCS', 'CLK', 'BK1_IO3', 'BK1_IO2', 'BK1_IO1', 'BK1_IO0')
	{
		(exists $Conf{$i}) && ($line .= sprintf("%s: %s, ", $Conf{$i}, $i));
	}
	chop($line);
	chop($line);
	printf("${Sep}# %s\n", $line);
}

if (exists $Conf{'BK2_IO0'})
{
  /* QuadSPI on F4, F7, H7 */
	my $line;
	for my $i ('NCS', 'BK2_NCS', 'CLK', 'BK2_IO3', 'BK2_IO2', 'BK2_IO1', 'BK2_IO0')
	{
		(exists $Conf{$i}) && ($line .= sprintf("%s: %s, ", $Conf{$i}, $i));
	}
	chop($line);
	chop($line);
	printf("${Sep}# %s\n", $line);
}

if (exists $Conf{'P1_IO0'})
{
  /* OctoSPI on L4+ */
	my $line;
	for my $i ('P1_NCS', 'P1_CLK', 'P1_DQS', 'P1_IO7', 'P1_IO6', 'P1_IO5', 'P1_IO4',
			   'P1_IO3', 'P1_IO2', 'P1_IO1', 'P1_IO0')
	{
		(exists $Conf{$i}) && ($line .= sprintf("%s: %s, ", $Conf{$i}, $i));
	}
	chop($line);
	chop($line);
	printf("${Sep}# %s\n", $line);
}

if (exists $Conf{'P2_IO0'})
{
  /* OctoSPI on L4+ */
	my $line;
	for my $i ('P2_NCS', 'P2_CLK', 'P2_DQS', 'P2_IO7', 'P2_IO6', 'P2_IO5', 'P2_IO4',
			   'P2_IO3', 'P2_IO2', 'P2_IO1', 'P2_IO0')
	{
		(exists $Conf{$i}) && ($line .= sprintf("%s: %s, ", $Conf{$i}, $i));
	}
	chop($line);
	chop($line);
	printf("${Sep}# %s\n", $line);
}

my @conf = split(/\s*,\s*/, $Conf);
foreach my $line (@conf)
{
	$line = uc($line);
	$line =~ /^P([A-K])([0-9]+):\s*([A-Z0-9]+):(L|M|H|V)$/;
	my $port = $1;
	my $pin = $2;
	my $conf = $3;
	my $speed = $4;

	my $MODER = 0x0;
	my $OTYPER = 0x0;
	my $OSPEEDR = (($speed eq 'V') ? 0x3 : (($speed eq 'H') ? 0x2 : (($speed eq 'M') ? 0x1 : 0x0)));
	my $PUPDR = 0x0;
	my $AFR = 0x0;
	my $num = ord(${port}) - ord('A');
	my $out = $Out[$num];

	($conf eq '') && next;
	(exists $$out{'DEF'}) || ($$out{'DEF'} = 0);

	if ($conf =~ /^AF([0-9]+)$/)
	{
		if (($1 < 0) || ($1 > 15))
		{
			printf(STDERR "invalid alternate %s\n", $line);
			next;
		}
		$MODER = 0x2;
		$AFR = $1;
		if ($pin <= 7)
		{
			$$out{'AFRL_H'} |= ($AFR << (${pin} << 2));
			$$out{'AFRL_L'} |= (($AFR ^ 0xF) << (${pin} << 2));
		}
		else
		{
			$$out{'AFRH_H'} |= ($AFR << ((${pin} - 8) << 2));
			$$out{'AFRH_L'} |= (($AFR ^ 0xF) << ((${pin} - 8) << 2));
		}
		$conf = sprintf("AF%02d", $AFR);
	}
	elsif ($conf =~ /^IN(|UP|DO)$/)
	{
		$MODER = 0x0;
		$PUPDR = ($1 eq 'UP') ? 0x1 : (($1 eq 'DO') ? 0x2 : 0x0);
		$$out{'PUPDR_H'} |= ($PUPDR << (${pin} << 1));
		$$out{'PUPDR_L'} |= (($PUPDR ^0x3) << (${pin} << 1));
	}
	elsif ($conf =~ /^P(P|O)(|UP|DO)$/)
	{
		$MODER = 0x1;
		$OTYPER = ($1 eq 'O') ? 0x1 : 0x0;
		$PUPDR = ($2 eq 'UP') ? 0x1 : (($2 eq 'DO') ? 0x2 : 0x0);
		$$out{'OTYPER_H'} |= ($OTYPER << ${pin});
		$$out{'OTYPER_L'} |= (($OTYPER ^ 0x1) << ${pin});
		$$out{'PUPDR_H'} |= ($PUPDR << (${pin} << 1));
		$$out{'PUPDR_L'} |= (($PUPDR ^0x3) << (${pin} << 1));
	}
	else
	{
		printf(STDERR "invalid conf %s\n", $line);
		next;
	}

	if ($$out{'DEF'} & (1<< ${pin}))
	{
		printf(STDERR "redefinition: %s\n", $line);
	}
	$$out{'DEF'} |= (1 << ${pin});
	$$out{'MODER_H'} |= ($MODER << (${pin} << 1));
	$$out{'MODER_L'} |= (($MODER ^ 0x3) << (${pin} << 1));

	$$out{'OSPEEDR_H'} |= ($OSPEEDR << (${pin} << 1));
	$$out{'OSPEEDR_L'} |= (($OSPEEDR ^ 0x3) << (${pin} << 1));

	push(@{$Port[$num]}, sprintf("P%s%02d:%s:%s", $port, $pin, $conf, $speed));
}

my @Col;
my $Set;
for (my $i = 0; $i < @Out; $i++)
{
	my $out = $Out[$i];
	my $addr = ${GPIO_BASE} + $i * ${GPIO_OFFS};
	my $count = 0;

	if (($$out{'MODER_H'} | $$out{'MODER_L'} | $$out{'OTYPER_H'} | $$out{'OTYPER_L'} |
			 $$out{'OSPEEDR_H'} | $$out{'OSPEEDR_L'} | $$out{'PUPDR_H'} | $$out{'PUPDR_L'} |
			 $$out{'AFRL_H'} | $$out{'AFRL_L'} | $$out{'AFRH_H'} | $$out{'AFRH_L'}) != 0)
	{
		push(@Col, sort({ $b cmp $a } @{$Port[$i]}));

		$Set .= sprintf("%s# Port %s: %s\n", ${Sep}, chr($i + ord('A')),
			join(", ", sort({ $b cmp $a } @{$Port[$i]})));

		if (($$out{'MODER_H'} | $$out{'MODER_L'}) != 0)
		{
			$Set .= sprintf("${Form}MODER\n", $addr + ${GPIO_MODER}, $$out{'MODER_H'}, $$out{'MODER_L'});
		}

		if (($$out{'OTYPER_H'} | $$out{'OTYPER_L'}) != 0)
		{
			$Set .= sprintf("${Form}OTYPER\n", $addr + ${GPIO_OTYPER}, $$out{'OTYPER_H'}, $$out{'OTYPER_L'});
		}

		if (($$out{'OSPEEDR_H'} | $$out{'OSPEEDR_L'}) != 0)
		{
			$Set .= sprintf("${Form}OSPEEDR\n", $addr + ${GPIO_OSPEEDR}, $$out{'OSPEEDR_H'}, $$out{'OSPEEDR_L'});
		}

		if (($$out{'PUPDR_H'} | $$out{'PUPDR_L'}) != 0)
		{
			$Set .= sprintf("${Form}PUPDR\n", $addr + ${GPIO_PUPDR}, $$out{'PUPDR_H'}, $$out{'PUPDR_L'});
		}

		if (($$out{'AFRL_H'} | $$out{'AFRL_L'}) != 0)
		{
			$Set .= sprintf("${Form}AFRL\n", $addr + ${GPIO_AFRL}, $$out{'AFRL_H'}, $$out{'AFRL_L'});
		}

		if (($$out{'AFRH_H'} | $$out{'AFRH_L'}) != 0)
		{
			$Set .= sprintf("${Form}AFRH\n", $addr + ${GPIO_AFRH}, $$out{'AFRH_H'}, $$out{'AFRH_L'});
		}

		$Set .= sprintf("\n");
	}
}

my $Col;
for (my $i = 0; $i < @Col; $i++)
{
	if (($i % 6) == 0)
	{
		printf("${Sep}# ", $Col);
	}
	$Col .= sprintf("%s, ", $Col[$i]);
	if (($i % 6) == 5)
	{
		chop($Col);
		chop($Col);
		printf("%s\n", $Col);
		$Col = '';
	}
}
chop($Col);
chop($Col);
($Col ne '') && ($Col .= "\n");
printf("%s\n%s", $Col, $Set);

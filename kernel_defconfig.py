#!/usr/bin/env python

"""
 Copyright (C) 2014 BlackBerry Limited

 This software is licensed under the terms of the GNU General Public
 License version 2, as published by the Free Software Foundation, and
 may be copied, distributed, and modified under those terms.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
"""

import sys
import optparse
from collections import OrderedDict

newline = '\n'
IS_NOT_SET = 'is not set'

def main( args ):

	p = optparse.OptionParser( usage="kernel_defconfig --input <defconfig input file> --override <defconfig override file> -- output <defconfig output file>" )

	p.add_option( '-i', '--input',
                 dest='input', action='store',
                 help="Defconfig input file name" )

	p.add_option( '-v', '--overlay',
                 dest='overlay', type='string',
                 action='callback',
                 callback=overlay_callback,
                 help="Defconfig overlay file name" )

	p.add_option( '-o', '--output',
                 dest='output', action='store',
                 help="Defconfig output file name" )

	opt, args = p.parse_args( args )
	if args:
		p.print_usage()
		sys.exit( 1 )

	oconfig = OrderedDict()
	for o in opt.overlay:
		extract_config( o, oconfig )
	iconfig = OrderedDict()
	extract_config( opt.input, iconfig )

	# Merge the overrides on top of the input config
	iconfig.update( oconfig )

	dump_config( iconfig, opt.output, opt.input, opt.overlay )

def overlay_callback( option, opt, value, parser ):
  setattr( parser.values, option.dest, value.split( ';' ) )

def extract_config( file, config ):

	with open( file, 'r' ) as ifile:
		lines = ifile.read().splitlines()

	# Store all config params in a dict

	for line in lines:
		line = line.strip()
		if line == '':
			continue
		elif line.startswith( '#' ):
			# "# CONFIG_X is not set" lines need to be preserved as they are treated as a no for that flag internally
			if line.startswith( '# CONFIG_' ) and line.endswith( IS_NOT_SET ):
				c = line.split( ' ' )[1]
				config[c] = IS_NOT_SET
			else:
				continue
		else:
			c = line.split( '=' )
			config[c[0].strip()] = c[1].strip()

def dump_config( config, file, ifile, ofiles ):

	with open( file, 'w+' ) as outfile:

		outfile.write( '# This is a generated file, DO NOT EDIT manually' + newline )
		outfile.write( '# Source: ' + ifile + newline )
		outfile.write( '# Overlays: ' + ' '.join( ofiles ) + newline )

		for k in config.keys():
			if config[k] == IS_NOT_SET:
				outfile.write( '# ' + k + ' ' + config[k] + newline )
			else:
				outfile.write( k + '=' + config[k] + newline )

if __name__ == '__main__':
  main( sys.argv[1:] )

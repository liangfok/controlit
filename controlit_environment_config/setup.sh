#!/bin/bash

cur_dir=$(dirname $BASH_SOURCE)

hostname=`hostname -s`

# Ordering is important
# setup_list="vars functions git rttlua ros aliases"
setup_list="vars"

for script in $setup_list
do
	source $cur_dir/setup/$script.sh
done

# export-append PATH $cur_dir/scripts

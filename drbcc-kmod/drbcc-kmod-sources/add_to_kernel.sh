#!/bin/bash
#
# Developer script to patch monolithic (i.e. non-module, in-kernel) drbcc driver support
#  into a kernel source tree.
#
# Do this:
# $ bitbake linux
# $ cd OE/tmp.6/work/hipox-angstrom-linux-gnueabi/linux-2.6.24-r<RECIPE_VERSION>/linux-2.6.24/
# $ ../../../../../../LINUX/drbcc-kmod/add_to_kernel.sh
#
# Then:
# $ ../temp/run.do_compile.<SOME_NUMBER>
#
# You will interactively be asked whether (and how) to include the bcc code into the kernel build by the kernel config / compile system on the command line.
#

WORK_DIR=$(pwd)
DRBCC_PATH=$(dirname ${BASH_SOURCE[0]})

echo -e "WORK HERE: $WORK_DIR\nSOURCES HERE: $DRBCC_PATH"

bcc_files_req="bcc-in-kernel-tree.patch  debug.h  drbcc-core.c  drbcc.h drbcc_ll.h  drbcc_packet.c  drbcc_raw.c  drbcc_rtc.c  Makefile"
bcc_files_actual="`ls -la $DRBCC_PATH` fail"

# at first some sanity checks:

# - alle BCC-Quellen und Kernel-Patch vorhanden in $DRBCC_PATH?
for file in $bcc_files_req; do 
	for tmp_file in $bcc_files_actual; do
		if [ "$file" == "$tmp_file" ]; then
			break;
		fi
	done
	if [ "$tmp_file" == "fail" ]; then
		echo "File $file not found in $DRBCC_PATH."
		exit -1;
	fi
done

# - Verzeichnis linux-2.4.24/drivers vorhanden in $work_dir?
if [ ! -d $WORK_DIR/drivers ]; then
	echo "Directory $WORK_DIR/drivers doesn't exist."
	exit -1;
fi

if [ ! -d $WORK_DIR/drivers/drbcc ]; then
	echo "Creating directory $WORK_DIR/drivers/drbcc."
	mkdir $WORK_DIR/drivers/drbcc
fi

echo "Copy files from ~/src/hyp-trunk/LINUX/drbcc-kmod/ to $PWD/drivers/drbcc/."
cp $DRBCC_PATH/*.[hc] $WORK_DIR/drivers/drbcc/
cp $DRBCC_PATH/Makefile $WORK_DIR/drivers/drbcc/
cp $DRBCC_PATH/bcc-in-kernel-tree.patch $WORK_DIR/drivers/drbcc/

echo "Patch kernel."
ret=`patch -s -p1 --dry-run < $DRBCC_PATH/bcc-in-kernel-tree.patch`

if [ -n "$ret" ]; then 
	echo "Patching Kernel seems to fail in dry-run: "
	echo $ret
	exit -1;
else 
	patch -p1 < $DRBCC_PATH/bcc-in-kernel-tree.patch
fi

# tfm: disabled because .config is currently missing from the repository
# echo -n "Generate .config with monolithic drbcc components enabled? [Y/n] "
# read genconf
# echo $genconf
# if [[ "$genconf" != "n" && "$genconf" != "N" ]]; then
# 	echo "Copy kernel .config from $DRBCC_PATH to $WORK_DIR"
# 	cp $DRBCC_PATH/.config $WORK_DIR
# fi

exit 0;

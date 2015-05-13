#!/bin/sh
COMPILE_QUARTUS=0
#
# Resolve TOP_LEVEL_DIR, default to PWD if no path provided.
#
if [ $# -eq 0 ]; then
TOP_LEVEL_DIR=$PWD
else
TOP_LEVEL_DIR=$1
fi
echo "TOP_LEVEL_DIR is $TOP_LEVEL_DIR"
echo
#
# Generate SOPC list...
#
SOPC_LIST=`find $TOP_LEVEL_DIR -name "*.sopc"`
#
# Generate Quartus II project list.
#
PROJ_LIST=`find $TOP_LEVEL_DIR -name "*.qpf" | sed s/\.qpf//g`
#
# Main body of the script. First "generate" all of the SOPC Builder
# systems that are found, then compile the Quartus II projects.
#
#
# Run SOPC Builder to "generate" all of the systems that were found.
#
for SOPC_FN in $SOPC_LIST
do
cd `dirname $SOPC_FN`
if [ ! -e `basename $SOPC_FN .sopc`.vhd -a ! -e `basename $SOPC_FN .sopc`.v ]; then
echo; echo
echo "INFO: Generating $SOPC_FN SOPC Builder system."
sopc_builder -s --generate=1 --no_splash
if [ $? -ne 4 ]; then
echo; echo
echo "ERROR: SOPC Builder generation for $SOPC_FN has failed!!!"
echo "ERROR: Please check the SOPC file and data " \
"in the directory `dirname $SOPC_FN` for errors."
fi
else
echo; echo
echo "INFO: HDL already exists for $SOPC_FN, skipping Generation!!!"
fi
cd $TOP_LEVEL_DIR
done
if [ $COMPILE_QUARTUS ]; then
for PROJ in $PROJ_LIST
do
cd `dirname $PROJ`
if [ ! -e `basename $PROJ`.sof ]; then
echo; echo
echo "INFO: Compiling $PROJ Quartus II Project."
quartus_cmd `basename $PROJ`.qpf -c `basename $PROJ`.qsf
if [ $? -ne 4]; then
echo; echo
echo "ERROR: Quartus II compilation for $PROJ has failed!!!."
echo "ERROR: Please check the Quartus II project “ \
“in `dirname $PROJ` for details."
fi
else
echo; echo
echo "INFO: SOF already exists for $PROJ, skipping compilation."
fi
cd $TOP_LEVEL_DIR
done
fi
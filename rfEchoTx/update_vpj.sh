#!/bin/sh

#set -x

VPJ_FILE=../vs/rfEchoTx.vpj

dot_d_2vs.sh ${VPJ_FILE} Debug
cat ${VPJ_FILE} | \
sed -e 's!<F N="../rfEchoTx/!<F N="../!g' | \
sed -e 's!<F N="../syscfg/!<F N="../Debug/syscfg/!g' | \
sed -e 's!<F N="../\([^.]\)!<F N="../rfEchoTx/\1!g' \
> e.xml
mv e.xml ${VPJ_FILE}


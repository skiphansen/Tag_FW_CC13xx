#!/bin/sh

#set -x

VPJ_FILE=../vs/rfEchoTx_CC1310.vpj

dot_d_2vs.sh ${VPJ_FILE} Debug
cat ${VPJ_FILE} | \
sed -e 's!<F N="../rfEchoTx_CC1310/!<F N="../!g' | \
sed -e 's!<F N="../syscfg/!<F N="../Debug/syscfg/!g' | \
sed -e 's!<F N="../\([^.]\)!<F N="../rfEchoTx_CC1310/\1!g' | \
sed -e 's!<F N="../rfEchoTx_CC1310/shared/\([^.]\)!<F N="../shared/\1!g' | \
sed -e 's!<F N="../rfEchoTx_CC1310/common/\([^.]\)!<F N="../common/\1!g' \
> e.xml
mv e.xml ${VPJ_FILE}


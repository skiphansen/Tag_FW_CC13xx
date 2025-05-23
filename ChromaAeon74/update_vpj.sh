#!/bin/sh

VPJ_FILE=../vs/ChromaAeon74.vpj

dot_d_2vs.sh ${VPJ_FILE} Debug
cat ${VPJ_FILE} | \
sed -e 's!<F N="../ChromaAeon74/syscfg/!<F N="../Debug/syscfg/!g' | \
sed -e 's!<F N="../\([^.]\)!<F N="../ChromaAeon74/\1!g' | \
sed -e 's!<F N="../ChromaAeon74/shared/\([^.]\)!<F N="../shared/\1!g' | \
sed -e 's!<F N="../ChromaAeon74/common/\([^.]\)!<F N="../common/\1!g' | \
sed -e 's!<F N="../ChromaAeon74/uzlib/\([^.]\)!<F N="../uzlib/\1!g' | \
sed -e 's!<F N="../ChromaAeon74/bb_epaper/\([^.]\)!<F N="../bb_epaper/\1!g' | \
sed -e 's!<F N="../ChromaAeon74/ChromaAeon74/\([^.]\)!<F N="../ChromaAeon74/\1!g'  \
> e.xml
mv e.xml ${VPJ_FILE}


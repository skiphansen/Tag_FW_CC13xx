#!/bin/sh

VPJ_FILE=vs/ChromaAeon74.vpj

dot_d_2vs.sh ${VPJ_FILE} Debug
cat ${VPJ_FILE} | sed -e 's!<F N="../../\([^.]\)!<F N="../\1!g' | \
sed -e 's!<F N="../syscfg/!<F N="../Debug/syscfg/!g' > e.xml
mv e.xml ${VPJ_FILE}


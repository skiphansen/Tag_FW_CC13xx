#!/bin/sh

VPJ_FILE=../vs/proxy_CC1311.vpj

dot_d_2vs.sh ${VPJ_FILE} Debug
cat ${VPJ_FILE} | \
sed -e 's!<F N="../proxy_CC1311/syscfg/!<F N="../Debug/syscfg/!g' | \
sed -e 's!<F N="../\([^.]\)!<F N="../proxy_CC1311/\1!g' | \
sed -e 's!<F N="../proxy_CC1311/shared/\([^.]\)!<F N="../shared/\1!g' | \
sed -e 's!<F N="../proxy_CC1311/common/\([^.]\)!<F N="../common/\1!g' | \
sed -e 's!<F N="../proxy_CC1311/proxy_CC1311/\([^.]\)!<F N="../proxy_CC1311/\1!g'  \
> e.xml
mv e.xml ${VPJ_FILE}


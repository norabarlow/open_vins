#!/bin/bash

files=`grep -Ril $1 ./ov_*/src/*`
for f in $files; do
#    echo $f && sed -i '' -e 's/double/float/g' $f && sed -i '' -e 's/2d/2f/g' $f
#    sed -i '' -e 's/2f/2d/g' $f
    echo $f
    sed -i '' -e "s/$1/$2/g" $f
done

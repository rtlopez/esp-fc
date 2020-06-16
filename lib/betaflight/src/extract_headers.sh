#!/bin/bash

FILES=$(grep -R '#include "' blackbox | sed -e 's/.*"\(.*\)"/\1/' | uniq)

for f in $FILES 
do
  DIR="$(dirname $f)"
  if [ $DIR != "." ]
  then
    mkdir -p $DIR
    touch $f
    echo "create $f"
  fi
done


#!/bin/bash

if [[ -z "$@" ]]; then
  echo "$0 <node/device> <signal> <value> <type> <dimensions>"
  exit 1
fi
type=$4
if [[ -z "$type" ]]; then
type=int
fi
dimensions=$5
if [[ -z "$dimensions" ]]; then
dimensions=1
fi

echo "set signal value on /io/$1 signal $2 = [ $3 ] { type: $type, dimensions [ $dimensions ]"
rostopic pub -r 100 /io/$1/command intera_core_msgs/IOComponentCommand '{  time : now,  op : "set", args : "{ \"signals\" : { \"'$2'\" : { \"format\" : {  \"type\" : \"'$type'\",  \"dimensions\" : [ '$dimensions' ] }, \"data\" : [ '$3' ] } } }" }'


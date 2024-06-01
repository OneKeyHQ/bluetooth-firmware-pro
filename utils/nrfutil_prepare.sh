#!/bin/bash
# $1 -> temp offline repo path

# make this script is because neither cmake or nrfutil is able to detect if the job is already done 
# thus everytime it will try install the tool slow down building process and may stuck on bad network

echo NRF Util Repo $1

if [ ! -d "$1" ]; then
    echo NRF Util Repo -\> Setup
    $(dirname $0)/nrfutil prepare-offline "$1"
fi

if [[ ! $($(dirname $0)/nrfutil list) =~ "nrf5sdk-tools" ]]; then
    echo NRF Util Repo -\> Install nrf5sdk-tools
    $(dirname $0)/nrfutil install nrf5sdk-tools --from-offline $1
fi


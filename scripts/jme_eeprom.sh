#!/bin/bash

########################################
# Getting eeprom data                  #
########################################

# Check bash version
msg="Please use bash version 3 or later."
if [[ ${BASH_VERSION} =~ ^([0-9])\.[0-9]\.(.*) ]]; then
	if (( ${BASH_REMATCH[1]} < 3 )); then
		echo ${msg}
		exit
	fi
else
	echo ${msg}
	exit
fi

# Parse arguments
if [[ ! $1 =~ ^eth[0-9]$ ]]; then
	echo "Usage: $0 ethX"
	echo
	echo "Print eeprom info of JME Ethernet."
	echo
	exit
fi

# Get eeprom data via ethtool
PATH="${PATH}:/sbin:/usr/sbin:/bin:/usr/bin"
romdata=$(ethtool -e $1 raw on |\
          od --width=1 --format=x1 --address-radix=x --output-duplicates |\
	  awk '{print $2;}')
if [[ -z "${romdata}" ]]; then
	exit
fi
romdata=$(echo ${romdata} | tr '[:lower:]' '[:upper:]' | sed 's/\( FF\)\+$//')

########################################
# Check for all non-optional data      #
########################################

# Check JME eeprom prefix
echo -n "EEPROM prefix check: "
if [[ ${romdata} =~ ^55\ AA\ (.*)$ ]]; then
	echo OK.
else
	echo "Failed. (Not JMicron NIC?)"
	exit
fi
romdata=${BASH_REMATCH[1]}

# Check JME eeprom suffix
echo -n "EEPROM suffix check: "
if [[ ${romdata} =~ ^(.*)\ 80\ 00\ 00$ ]]; then
	echo OK.
else
	echo "Failed (Not JMicron NIC?)."
	exit
fi
romdata=${BASH_REMATCH[1]}

# Check JME eeprom MAC Address
echo -n "EEPROM MAC Address check: "
if [[ ${romdata} =~ ^(.*)[\ ]?01\ 38\ ([0-9A-F]{2})\ 01\ 39\ ([0-9A-F]{2})\ 01\ 3A\ ([0-9A-F]{2})\ 01\ 3B\ ([0-9A-F]{2})\ 01\ 3C\ ([0-9A-F]{2})\ 01\ 3D\ ([0-9A-F]{2})[\ ]?(.*)$ ]]; then
	echo OK.
	MACADDR=${BASH_REMATCH[2]}:${BASH_REMATCH[3]}:${BASH_REMATCH[4]}:${BASH_REMATCH[5]}:${BASH_REMATCH[6]}:${BASH_REMATCH[7]}
	echo "       MAC Address: ${MACADDR}"
else
	echo Failed.
	exit
fi
romdata="${BASH_REMATCH[1]} ${BASH_REMATCH[8]}"

########################################
# Print out all optional data         #
########################################

if [[ -z "${romdata}" ]]; then
	exit
fi

# Print other eeprom data
(( smicmd = 0 ))
echo -e "\nOther eeprom data:"
while [[ ! -z "${romdata}" ]]; do
	if [[ ! ${romdata} =~ ^[\ ]?(0[012])\ ([0-9A-F]{2})\ ([0-9A-F]{2})[\ ]?(.*) ]]; then
		echo "    EEPROM contains error data"
		exit
	fi
	cmd=${BASH_REMATCH[1]}
	address=${BASH_REMATCH[2]}
	value=${BASH_REMATCH[3]}
	romdata=${BASH_REMATCH[4]}

	if [[ (( ${smicmd} == 0 )) && ${cmd} == "01" && ${address} == "53" ]]; then
		(( ++smicmd )); v1=${value}
	elif [[ (( ${smicmd} == 1 )) && ${cmd} == "01" && ${address} == "52" ]]; then
		(( ++smicmd )); v2=${value}
	elif [[ (( ${smicmd} == 2 )) && ${cmd} == "01" && ${address} == "51" ]]; then
		(( ++smicmd )); v3=${value}
	elif [[ (( ${smicmd} == 3 )) && ${cmd} == "01" && ${address} == "50" ]]; then
		(( smicmd = 0 ))
		v4=${value}
		data=${v1}${v2}
		bcenv="scale = 0; obase = 10; ibase = 16;"
		reg=$(echo "${bcenv} ${v3} / 8;" | bc)
		phy=$(echo "${bcenv} a = ${v3}; b = ${v4};\
			    a %= 4; a *= 8; b /= 40; a + b;" | bc)
		rw=$(echo "${bcenv} a = ${v4}; a /= 20; a % 2;" | bc)
		if [[ ${rw} == "1" ]]; then
			rw="Write"
			value=" value=${data}"
		else
			rw="Read"
			value=""
		fi
		valid=$(echo "${bcenv} a = ${v4}; a % 20;" | bc)
		if [[ ${valid} == "16" ]]; then
			valid="Valid"
		else
			valid="Invalid"
		fi
		echo "${valid} SMI Command: ${rw} phyaddr=${phy} reg=${reg}${value}"
	elif [[ (( ${smicmd} != 0 )) ]]; then
		echo "    EEPROM contains error data"
		exit
	else
		if [[ ${cmd} == "00" ]]; then
			echo -n "Configuration Space: "
		elif [[ ${cmd} == "01" ]]; then
			echo -n "First IO Space(BAR2): "
		elif [[ ${cmd} == "02" ]]; then
			echo -n "Second IO Space(BAR3): "
		fi
		echo "Address=${address} Value=${value}"
	fi
done


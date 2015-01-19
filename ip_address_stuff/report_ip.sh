#!/bin/bash
# echo "hi"
hello="hello world"
ip_address=`ifconfig $1 | grep "inet addr:18." | awk -F: '{print $2}' | awk '{print $1}'`
echo "edison IP:" + "$ip_address" > ipaddress.txt

# function int-ip { ifconfig $1 | grep "inet addr:18." | awk -F: '{print $2}' | awk '{print $1}'; }
# echo int-ip

# curl --data "ipaddress=$ip_address" teamup.xvm.mit.edu/report-ip.php

curl --url "smtps://smtp.gmail.com:465" --ssl-reqd \
  --mail-from "sandroraspi@gmail.com" --mail-rcpt "sandro1321@gmail.com" \
  --upload-file ipaddress.txt --user "sandroraspi@gmail.com:trumpet13" --insecure
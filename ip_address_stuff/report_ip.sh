#!/bin/bash

ip_address=ifconfig $1 | grep "inet addr:18." | awk -F: '{print $2}' | awk '{print $1}'
curl --data "ipaddress=$ip_address" edisonreceiver.xvm.mit.edu/report-ip.php
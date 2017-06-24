IP=$1
MYIP=$(route get $IP | grep "interface: .*" -o | awk '{print $2;}')
ifconfig $MYIP | egrep '[[:digit:]]{1,3}\.[[:digit:]]{1,3}\.[[:digit:]]{1,3}\.[[:digit:]]{1,3} ' -o -m 1

IP=$1
MYIP=$(ip route get $IP | grep "dev .*" -o | awk '{print $2;}')
ifconfig $MYIP | egrep '[[:digit:]]{1,3}\.[[:digit:]]{1,3}\.[[:digit:]]{1,3}\.[[:digit:]]{1,3} ' -o | head -1

# Colin T. Ward
# M.X. Grey ( mxgrey@gatech.edu )
# CRITR development

export LD_LIBRARY_PATH=/usr/lib:

#SERVER_IP='104.131.172.175'
SERVER_IP='127.0.0.1'

CRITR_REF='critr-ref'

MakeAch()
{

	ach mk $CRITR_REF -m 10 -n 64
   	sudo chmod 777 /dev/shm/achshm-*
}
Client_Remote()
{
	achd -d -r push $SERVER_IP $CRITR_REF
	sudo python Eclient.py
}
Server_Remote()
{
	achd -d -r pull $SERVER_IP $CRITR_REF
	sudo python Eserver.py	
}
MakeAch
Client_Remote

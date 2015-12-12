# Colin T. Ward
# M.X. Grey ( mxgrey@gatech.edu )
# CRITR development

export LD_LIBRARY_PATH=/usr/lib:

SERVER_IP='104.131.172.175'
CLIENT_IP='192.168.1.179'

SCLIENT='critr-ref'

MakeAch()
{

        ach mk $SCLIENT -m 10 -n 64	
        sudo chmod 777 /dev/shm/achshm-*
	#python critr_comm_test.py 
}
Client_Remote()
{
        MakeAch
        achd -r pull $SSERVER_IP $SSERVER
}
Server_Remote()
{
        MakeAch
        achd -r pull $SCLIENT_IP $SCLIENT
}
MakeAch
#!/usr/bin/awk -f
BEGIN{
	i=0
}
/broadcast D/{
	pktNum[i++]=$6
	recvNum[$6]=0
	dropNum[$6]=0
	uselessNum[$6]=0
	usefulNum[$6]=0
}
/ Received D/{
	x=$5
	recvNum[x]++
	getline;
	if (match ($0,"Dropped")){
		dropNum[x]++
	}
	if (match($0, "Useless")){
		uselessNum[x]++
	}
	if (match($0,"NID:")){
		usefulNum[x]++
	}
}
END{
	for (pkt in recvNum) {
		print pkt,",", recvNum[pkt],",", dropNum[pkt],",", uselessNum[pkt]
	}
}

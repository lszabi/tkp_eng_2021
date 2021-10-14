#!/bin/bash
echo "+-------------------------------------+"
echo "| Start TORCS Server                  |"
echo "+-------------------------------------+"
if [ -z ${CAR_NAME+x} ]; then CAR_NAME=car1-trb1; fi
echo "+-------------------------------------+"
echo "| RACE: ${RACE_FILE}"
echo "| ROAD NAME: ${ROAD_NAME}"
echo "| CAR NAME: ${CAR_NAME}"
echo "+-------------------------------------+"

echo "+-------------------------------------+"
echo "| Set Road                            |"
echo "+-------------------------------------+"

xmlstarlet edit  -L \
    --update "/params[@name='Quick Race']/section[@name='Tracks']/section[@name='1']/attstr[@name='name']/@val" \
    --value  "${ROAD_NAME}" ${RACE_FILE}

echo "+-------------------------------------+"
echo "| Set Car                             |"
echo "+-------------------------------------+"
xmlstarlet edit  -L \
    -u "/params[@name='scr_server']/section[@name='Robots']/section[@name='index']/section[@name='0']/attstr[@name='car name']/@val" \
    -v "${CAR_NAME}" /torcs/BUILD/share/games/torcs/drivers/scr_server/scr_server.xml

SRC_ROAD_DIR="torcs_road"
DST_ROAD_DIR="/torcs/BUILD/share/games/torcs/tracks/road"
if [ -d "$SRC_ROAD_DIR" ]
then
	if [ "$(ls -A $SRC_ROAD_DIR)" ]; then
        echo "+-------------------------------------+"
        echo "| Take action $SRC_ROAD_DIR is not Empty, copy roads"
        echo "+-------------------------------------+"
        cp -a $SRC_ROAD_DIR/. $DST_ROAD_DIR
	else
    echo "+-------------------------------------+"
    echo "| $SRC_ROAD_DIR is Empty"
    echo "+-------------------------------------+"
	fi
else
    echo "+-------------------------------------+"
	echo "| Directory $SRC_ROAD_DIR not found."
    echo "+-------------------------------------+"
fi

torcs/BUILD/bin/torcs -r  ${RACE_FILE}

chmod -R 777 /root/.torcs
chmod -R 777 /torcs_road

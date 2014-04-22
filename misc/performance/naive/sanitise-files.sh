#!/bin/bash
# This script will take in a .txt file containing the ROS output of our pose_estimator
# and randomly pick 10 rows of each type to dump into a text file.
# It assumes that there is are no empty or erroneous lines in the file.
# Can be used with find to automate the process, e.g.:
# find . -name "*.txt" -exec ./sanitise-files.sh {} \;

FILENAME=$1

# minimum line check
LINES=$[`wc -l $1 | awk '{ print $1 }'`]
if [ $LINES -lt "40" ]; then
 	echo "$FILENAME does not have the minimum number of lines"
	exit 1;
fi

# remove log timestamp
echo $FILENAME
awk -F ":" '{ print $2 }' $FILENAME > $FILENAME.tmp

# grab headers
awk 'if NR==1' $FILENAME.tmp > overall-$FILENAME
awk 'if NR==2' $FILENAME.tmp > detectCorners-$FILENAME

# grab relevant rows, get 10 randomly using shuf
awk 'NF==7 && NR%2==0' $FILENAME.tmp | shuf -n 10 >> overall-$FILENAME
awk 'NF==8 && NR%2==0' $FILENAME.tmp | shuf -n 10 >> detectCorners-$FILENAME

# clean up
rm $FILENAME.tmp
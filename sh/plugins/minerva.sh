export MINERVA_INSTNCE_ID=i-0f208718ebb57563f

start-instance () {
  aws ec2 start-instances --instance-ids $MINERVA_INSTNCE_ID
}

describe-instance () {
  aws ec2 describe-instances --instance-ids $MINERVA_INSTNCE_ID
}

stop-instance () {
  aws ec2 stop-instances --instance-ids $MINERVA_INSTNCE_ID
}

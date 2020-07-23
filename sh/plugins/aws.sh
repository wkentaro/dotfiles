profiles=(minerva corvus)

echo_bold () {
  echo -e "\033[1m$*\033[0m"
}

als() {
  for profile in $profiles; do
    echo "# $profile"
    aws ec2 describe-instances --profile $profile | jq '.Reservations[] | ( .Instances[] | {state: .State.Name, id: .InstanceId, name: .KeyName, type: .InstanceType, ip: .PublicIpAddress})' | jq_to_table
  done
}
ast() {
  for profile in ${profiles[@]}; do
    aws --profile $profile ec2 start-instances --instance-ids $1 2>/dev/null && break
  done
}
asp() {
  for profile in ${profiles[@]}; do
    aws --profile $profile ec2 stop-instances --instance-ids $1 2>/dev/null && break
  done
}

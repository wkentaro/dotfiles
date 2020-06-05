profiles=(minerva corvus)

regions=(us-east-1 eu-west-1)

echo_bold () {
  echo -e "\033[1m$*\033[0m"
}

als() {
  for profile in ${profiles[@]}; do
    echo_bold "# $profile"
    for region in ${regions[@]}; do
      content=$(awless --no-sync list instances -p $profile -r $region)
      if [ $(echo $content | wc -l) -ne 1 ]; then
        echo $content
      fi
    done
    echo
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

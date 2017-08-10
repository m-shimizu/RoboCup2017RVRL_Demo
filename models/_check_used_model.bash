mkdir ../used_models
grep "model:" ../src/rc2017rvrl/worlds/*.world | awk -F ':' '{print $3}' | awk -F '/' '{print $3}' | sed 's/<//' | sort | uniq | awk '{printf("mv %s ../used_models\n", $0)}' > aaa
source aaa
rm aaa

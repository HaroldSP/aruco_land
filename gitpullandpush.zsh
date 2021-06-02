#! /bin/zsh

NOW=$( date '+%F_%H:%M:%S' )

git pull

git add .
#git commit -m "$NOW"
git commit -m "$NOW ___ $1"
git push -u origin master 

#делаем принудительный коммит в основной репо на гитхабе
#git push -f origin master
#без -f будет ругаться что у вас версия младше чем в гитхабе и вам надо сделать pull

#passing arguments to a bash script:
#https://www.futurelearn.com/info/courses/linux-for-bioinformatics/0/steps/202962
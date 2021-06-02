#! /bin/zsh

NOW=$( date '+%F_%H:%M:%S' )

git add .
git commit -m "$NOW"
git push -u origin master 

#делаем принудительный коммит в основной репо на гитхабе
#git push -f origin master
#без -f будет ругаться что у вас версия младше чем в гитхабе и вам надо сделать pull
#!~\Documents\FU630Driver
echo Changing git branch to gradiantOptimization
git pull
git status
sleep 1
git checkout gradiantOptimization .
git pull -v
sleep 1
echo Moving OptDriverTst.py outside of driver folder
cd ~\Documents
cp -f -v FU630Driver\OptDriverTst.py OptDriverTst.py
rm -i -v OptDriverTstTst.py
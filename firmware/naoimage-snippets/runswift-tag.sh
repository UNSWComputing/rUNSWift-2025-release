

git -C /workspace rev-parse HEAD > ./root/nao/image.commit.sha
date > ./root/nao/image.build.time

cp ./root/nao/image.commit.sha ../../
cp ./root/nao/image.build.time ../../

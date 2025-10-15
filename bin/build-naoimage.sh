#!/bin/bash

# AARNet is usually fastest, though you can try http://au.archive.ubuntu.com/ubuntu, http://nz.archive.ubuntu.com/ubuntu, etc.
NAOIMAGE_APT_MIRROR="http://mirror.aarnet.edu.au/ubuntu"
ssh-keyscan runswift2.cse.unsw.edu.au >> ~/.ssh/known_hosts

# Note: This list is ordered, and the order matters. Keep runswift-tag last.
NAOIMAGE_SNIPPETS="ubuntu firmware-update ros2-base ros2-dependencies save-base runswift-dependencies runswift-base runswift-robotconfig runswift-tag"

# copy /workspace/firmware into ~/firmware
# This solves the permission issues as workspace is mounted
WORKSPACE_ROOT=$(git rev-parse --show-toplevel)
IMAGE_BUILDING_DIR=/tmp/BUILDING_NAO_IMAGE_DIR

OFFICIAL_OPN_FILE=$IMAGE_BUILDING_DIR/firmware/nao-2.8.5.11_ROBOCUP_ONLY_with_root.opn
NAOIMAGE_EXT3_OUTPUT=$IMAGE_BUILDING_DIR/firmware/nao-2.8.5.11-ubuntu-22.04-ros2.ext3
NAOIMAGE_OPN_OUTPUT=$WORKSPACE_ROOT/firmware/nao-2.8.5.11-ubuntu-22.04-ros2.opn
NAOIMAGE_DIR=$IMAGE_BUILDING_DIR/firmware/naoimage
NAOIMAGE_SNIPPETS_DIR=$IMAGE_BUILDING_DIR/firmware/naoimage-snippets

SECRETS_FILE=$IMAGE_BUILDING_DIR/secrets/secrets.env

if [ -f "$NAOIMAGE_OPN_OUTPUT" ]; then
    echo "OPN image already exists at $NAOIMAGE_OPN_OUTPUT, nothing to do."
    echo -n "Build anyway? (y/N): "
    read -r response
    if [[ ! "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        # exit with success
        exit 0
    fi
fi

# remove the old OPN if we are going to build a new one
# cleanup a half built system
for mount in $(mount | grep BUILDING_NAO_IMAGE_DIR | awk '{print $3}'); do
  sudo umount -f $mount
done
sudo mknod -m 666 /dev/null c 1 3
sudo mknod -m 666 /dev/zero c 1 5
sudo mknod -m 666 /dev/random c 1 8
sudo mknod -m 666 /dev/urandom c 1 9

rm $NAOIMAGE_OPN_OUTPUT
sudo rm -rf $IMAGE_BUILDING_DIR
mkdir -p $IMAGE_BUILDING_DIR
cp -rv $WORKSPACE_ROOT/firmware $IMAGE_BUILDING_DIR/
cp -rv $WORKSPACE_ROOT/secrets $IMAGE_BUILDING_DIR/
cp -rv $WORKSPACE_ROOT/image $IMAGE_BUILDING_DIR/
cp -rv $WORKSPACE_ROOT/robots $IMAGE_BUILDING_DIR/

if [ ! -f "$OFFICIAL_OPN_FILE" ]; then
    if [ ! -f "$SECRETS_FILE" ]; then
        echo "Error: The proprietary Nao kernel OPN image needs to be downloaded, but you don't have secrets.env."
        echo "       You can:"
        echo "       - Run \`make secrets\` to obtain secrets.env, then re-run \`make build-image\` to download from UNSW servers."
        echo "       - Alternatively, manually supply firmware/nao-2.8.5.11_ROBOCUP_ONLY_with_root.opn to proceed."
        echo
        exit 1
    fi

    source $SECRETS_FILE

    # download the official image from the CSE server
    echo "Downloading official OPN image from the CSE server..."
    sshpass -p "$REPOSITORY_SSH_PASSWORD" rsync -aP $REPOSITORY_SSH_URL:/var/www/html/opennao2/build-2.8.5.1x/nao-2.8.5.11_ROBOCUP_ONLY_with_root.opn $OFFICIAL_OPN_FILE
fi

if [ ! -d "$NAOIMAGE_DIR" ]; then
    mkdir -p $NAOIMAGE_DIR

    # clone the NaoImage repo
    git clone --depth=1 https://github.com/NaoDevils/NaoImage.git $NAOIMAGE_DIR

    # remove .git directory
    rm -rf $NAOIMAGE_DIR/.git
fi

# copy snippets to the NaoImage directory
cp -v $NAOIMAGE_SNIPPETS_DIR/*.sh $NAOIMAGE_DIR/snippets/

# switch to good mirrors for faster download
sed -i "s@http://de.archive.ubuntu.com/ubuntu@${NAOIMAGE_APT_MIRROR}@" $NAOIMAGE_DIR/snippets/ubuntu.sh

# change into the NaoImage directory
CURDIR=$PWD
cd $NAOIMAGE_DIR

# make a free loop device (necessary for mounting the image)
if [ ! -e /dev/loop0 ]; then
    sudo mknod /dev/loop0 b 7 0
fi

echo "Building the image... in $NAOIMAGE_DIR"
# build the image with the snippets
sudo ./generate_image.sh $OFFICIAL_OPN_FILE $NAOIMAGE_EXT3_OUTPUT $NAOIMAGE_SNIPPETS
echo "Done building the image!"
echo "The EXT3 image is located at $NAOIMAGE_EXT3_OUTPUT"
./generate_opn.sh $NAOIMAGE_EXT3_OUTPUT $IMAGE_BUILDING_DIR/firmware/temp.opn

cd $CURDIR

rm -rf $WORKSPACE_ROOT/firmware/naoimage
# cp -r $IMAGE_BUILDING_DIR/firmware/naoimage $WORKSPACE_ROOT/firmware/naoimage
cp -r $IMAGE_BUILDING_DIR/firmware/temp.opn $NAOIMAGE_OPN_OUTPUT
echo "Copied the OPN image from $IMAGE_BUILDING_DIR/firmware/temp.opn to $NAOIMAGE_OPN_OUTPUT"

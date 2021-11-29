TARGET=Mega2560
PORT=/dev/ttyUSB0
AVRDUDE_PROGRAMMER?=-cavr109
#SSH_UPLOAD_USER=respeaker
#SSH_UPLOAD_HOST=airtoo.local
#SSH_UPLOAD_HOST=192.168.8.209
GITHUB_REPOS=\
reeltwo/Reeltwo

include ../Arduino.mk

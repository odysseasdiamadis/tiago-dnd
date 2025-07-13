#!/bin/sh

apt update --yes
apt install mpg123 --yes

pip install transformers gtts flask openai-whisper
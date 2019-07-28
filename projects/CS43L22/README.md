Uses the audio dac CS43L22 stm32f4-discovery board
for first version we will figure out easy method to transmit data to headphone
jack then progressivly move towards using DMA. First priority is getting the
basic code then getting a basic version working.

TODO:
1. compile
2. I2C module double check
3. WR/RD register function  for CS43L22
4. I2S module coding
5. PLLI2S code

references:
http://www.mind-dump.net/configuring-the-stm32f4-discovery-for-audio
https://statics.cirrus.com/pubs/proDatasheet/CS43L22_F2.pdf
https://www.st.com/content/ccc/resource/technical/document/application_note/c7/2f/66/a5/cd/4c/4d/2a/DM00040802.pdf/files/DM00040802.pdf/jcr:content/translations/en.DM00040802.pdf
http://www.electroons.com/blog/hello-world/
https://sites.google.com/site/johnkneenmicrocontrollers/18b-i2c/i2c_stm32f407
https://hackaday.com/2019/04/18/all-you-need-to-know-about-i2s/
https://www.st.com/content/ccc/resource/technical/document/application_note/de/14/eb/51/75/e3/49/f8/DM00074956.pdf/files/DM00074956.pdf/jcr:content/translations/en.DM00074956.pdf
https://community.st.com/s/feed/0D50X00009XkW1mSAF
http://vedder.se/2012/12/stm32f4-discovery-usb-host-and-mp3-player/

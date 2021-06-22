# Code Explanation
1. CameraWebServerPermanent : An initial code with webserver and MQTT Send (Yoga & Martin)
2. FaceDetectionWithoutServer : A code without webserver (auto recognize without pressing the button), no MQTT Send (Ernie)
3. MQTT + Face Detection wo Server : A Combined between Code 1 and 2, left some errors (Martin)
4. CameraRecognize : Another code from Yoga which could detect face without a webserver but have to import the images manually on the partition
5. FaceRecog_with_LiftCounter : A combination between local version of face recognition and lift counter, no MQTT Send (Ernie)
6. Node-Red-Flow : The Node-Red flow that deal with the received messages and also send QUERY messages to AWS database, the testing flow is also included (John)

# Smart Dumbbell
*This is a student project of Cloud and Fog Internet of Things course, National Taiwan University of Science and Technology, Taiwan Tech, Taipei, Taiwan, Spring 2021*

## Team Member
Team 1 - NTUST CFIoT Team 2021
- 江承翰 John Chiang (B10632040)
- 蘇于銨 Ernie Su (B10632006)
- Bernard Mwangi (D10915815)
-  王瑜伽 I Wayan Wiprayoga Wisesa (D10902811)
- 馬努朗 Martin Clinton Manullang (D10902809)

## Architecture
<img width="626" alt="Screen Shot 2021-06-21 at 22 29 46" src="https://user-images.githubusercontent.com/22334778/122779222-40680c00-d2e0-11eb-9180-9c5c9d0c9125.png">
[Markdown - Link](#https://docs.google.com/spreadsheets/d/1kOLyi5Gj4014HxtFdDBxpM_J4166p6QUqVq4vnk0PhQ/edit?usp=sharing)
[Google Sheet Link](#https://docs.google.com/spreadsheets/d/1kOLyi5Gj4014HxtFdDBxpM_J4166p6QUqVq4vnk0PhQ/edit?usp=sharing)
  
## Video Demo
[![Video Demo](https://img.youtube.com/vi/8GxmKBJc71A/0.jpg)](https://www.youtube.com/watch?v=8GxmKBJc71A)

## Node-Red Configuration
<img width="367" alt="JSON" src="https://user-images.githubusercontent.com/22334778/122839666-0d973580-d32b-11eb-9dc9-df9bfd46b001.png">

## Schematic
![Lift_Counter_bb](https://user-images.githubusercontent.com/22334778/122847497-63bfa500-d33a-11eb-815f-ab26f2bb14fb.jpg)






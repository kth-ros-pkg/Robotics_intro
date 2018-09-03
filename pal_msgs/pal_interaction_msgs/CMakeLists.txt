cmake_minimum_required(VERSION 2.8.3)
project(pal_interaction_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
    actionlib_msgs
)


add_message_files(
  DIRECTORY msg
  FILES
    actiontag.msg
    ASRActivation.msg
    ASREvent.msg
    ASRLangModelMngmt.msg
    ASRLanguage.msg
    asrresult.msg
    ASRSrvRequest.msg
    ASRSrvResponse.msg
    ASRStatus.msg
    asrupdate.msg
    AudioDeviceDescription.msg
    AudioPlayerState.msg
    audiosignal.msg
    DirectionOfArrival.msg
    EnablingSoundLocalisation.msg
    I18nText.msg
    I18nArgument.msg
    TTSstate.msg
    TtsText.msg
    TtsMark.msg
    VoiceActivity.msg
    WebGuiEvent.msg
)

add_action_files(
  DIRECTORY action
  FILES
    ASRFile.action
    Sound.action
    Tts.action
)

add_service_files(
  DIRECTORY srv
  FILES
    ASRService.srv
    recognizerService.srv
    SoundLocalisationService.srv
)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

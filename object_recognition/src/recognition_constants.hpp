#ifndef RECOGNITION_CONSTANTS_HPP
#define RECOGNITION_CONSTANTS_HPP

#include <string>

namespace objRecognition
{

  /**
 * @brief The EObjectTypes enum represents all objects which can be detected
 *        (based on listing of pcd files)
 */
enum EObjectTypes
{
  OBJTYPE_NO_OBJECT = 0,            //< no object was detected
  OBJTYPE_UNKNOWN_OBJECT_DETECTED,  //< objected detected, but not identified
  OBJTYPE_AVOCADO,  //2
  OBJTYPE_BANANA,   //3
  OBJTYPE_BROCCOLI, //4
  OBJTYPE_CARROT,   //5
  OBJTYPE_CHILI,    //6
  OBJTYPE_CORN,     //7
  OBJTYPE_ELEPHANT, //8
  OBJTYPE_GIRAFFE,  //9
  OBJTYPE_GREEN_PUMPKIN,  //10
  OBJTYPE_HIPPO,    //11
  OBJTYPE_LEMON,    //12
  OBJTYPE_LION,     //13
  OBJTYPE_ONION,    //14
  OBJTYPE_PEACH,    //15
  OBJTYPE_PEAR,     //16
  OBJTYPE_POTATO,   //17
  OBJTYPE_TIGER,    //18
  OBJTYPE_TOMATO,   //19
  OBJTYPE_WATERMELON, //20
  OBJTYPE_ZEBRA,    //21
  OBJTYPE_RED_PLATE,  //22             //< no pcd file available
  COUNT_OBJTYPES,
};

/*
 * Corresponding string representation for EObjectTypes (all object types which can be detected)
 */
const std::string TEXT_OBJECTS[COUNT_OBJTYPES] =
{
  "No object detected.",
  "Object detected, but type not recognized yet.",
  "avocado",
  "banana",
  "broccoli",
  "carrot",
  "chili",
  "corn",
  "elephant",
  "giraffe",
  "green_pumpkin",
  "hippo",
  "lemon",
  "lion",
  "onion",
  "peach",
  "pear",
  "potato",
  "tiger",
  "tomato",
  "watermelon",
  "zebra",
  "red_plate",
};

}

#endif // RECOGNITION_CONSTANTS_HPP

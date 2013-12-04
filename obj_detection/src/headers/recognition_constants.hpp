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
  OBJTYPE_AVOCADO,
  OBJTYPE_BANANA,
  OBJTYPE_BROCCOLI,
  OBJTYPE_CARROT,
  OBJTYPE_CHILI,
  OBJTYPE_CORN,
  OBJTYPE_ELEPHANT,
  OBJTYPE_GIRAFFE,
  OBJTYPE_GREEN_PUMPKIN,
  OBJTYPE_HIPPO,
  OBJTYPE_LEMON,
  OBJTYPE_LION,
  OBJTYPE_ONION,
  OBJTYPE_PEACH,
  OBJTYPE_PEAR,
  OBJTYPE_POTATO,
  OBJTYPE_TIGER,
  OBJTYPE_TOMATO,
  OBJTYPE_WATERMELON,
  OBJTYPE_ZEBRA,
  OBJTYPE_RED_PLATE,              //< no pcd file available
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

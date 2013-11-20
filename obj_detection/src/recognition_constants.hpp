#ifndef RECOGNITION_CONSTANTS_HPP
#define RECOGNITION_CONSTANTS_HPP

#include <string>

namespace objRecognition {

  enum EObjectTypes {
    OBJTYPE_NO_OBJECT = 0,            //< no object was detected
    OBJTYPE_UNKNOWN_OBJECT_DETECTED,  //< objected detected, but not identified
    OBJTYPE_PEPERONI,
    OBJTYPE_TOMATO,
    OBJTYPE_RED_PLATE,
    OBJTYPE_CARROT,
    OBJTYPE_MELON,
    OBJTYPE_TIGER,
    OBJTYPE_GIRAFFE,
    OBJTYPE_HIPPO,
    OBJTYPE_PEAR,
    OBJTYPE_POTATO,
    OBJTYPE_LEMON,
    COUNT_OBJTYPES,
  };

  const std::string TEXT_OBJECTS[COUNT_OBJTYPES] = {
    "No object detected.",
    "Object detected, but type not recognized yet.",
    "Peperoni",
    "Tomato",
    "Red plate",
    "Carrot",
    "Melon",
    "Tiger",
    "Giraffe",
    "Hippo",
    "Pear",
    "Potato",
    "Lemon",
  };

}

#endif // RECOGNITION_CONSTANTS_HPP

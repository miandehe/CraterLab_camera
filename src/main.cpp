#if defined(ARDUINO_GIGA_M7)
  #include<main_m7.cpp>
#elif defined(ARDUINO_GIGA_M4)
  #include<main_m4.cpp>
#else
  #include<main_m7.cpp>
#endif
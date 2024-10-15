cmake_minimum_required(VERSION 3.22)
# L3G4200D/CMakeLists.txt

# Add the L3G4200D library
add_library(L3G4200D_HAL STATIC
            L3G4200D_HAL.c
)

# Specify include directories for this target
target_include_directories(L3G4200D_HAL PUBLIC 
    ${PROJECT_SOURCE_DIR}/User_Libraries/L3G4200D_STM32_HALDriver
)

target_link_libraries(L3G4200D_HAL stm32cubemx)
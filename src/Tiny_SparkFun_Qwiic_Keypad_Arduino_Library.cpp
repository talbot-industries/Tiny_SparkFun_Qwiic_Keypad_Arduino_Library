/*
  This is a library written for the SparkFun Qwiic Keypad
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/15168

  Written by Pete Lewis @ SparkFun Electronics, 3/12/2019
  Much of the code originally came from SparkX version,
  Located here: https://github.com/sparkfunX/Qwiic_Keypad

  The Qwiic Keypad is a I2C controlled 12 button keypad

  https://github.com/sparkfun/SparkFun_Qwiic_Keypad_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.8

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Tiny_SparkFun_Qwiic_Keypad_Arduino_Library.h"
#include <TinyWireM.h>

//Constructor
KEYPAD::KEYPAD(uint8_t bufferPin)
{
  _bufferPin = bufferPin;
}

//Initializes the I2C connection
//Returns false if board is not detected
bool KEYPAD::begin(uint8_t deviceAddress)
{
  _deviceAddress = deviceAddress;

  TinyWireM.begin(); //This resets any setClock() the user may have done
  return isConnected();
}

//Returns true if I2C device ack's
bool KEYPAD::isConnected()
{
  TinyWireM.beginTransmission(_deviceAddress);

  return TinyWireM.endTransmission() == 0;
}

bool KEYPAD::buttonAvailable()
{
  return digitalRead(_bufferPin) == LOW;
}

////Returns the button at the top of the stack (aka the oldest button)
byte KEYPAD::getButton()
{
  writeRegister(KEYPAD_UPDATE_FIFO, 0x01); // set bit0, commanding keypad to update fifo

  return readRegister(KEYPAD_BUTTON);
}

//Returns the number of milliseconds since the current button in FIFO was pressed.
uint16_t KEYPAD::getTimeSincePressed()
{
  uint16_t MSB = readRegister(KEYPAD_TIME_MSB);
  uint16_t LSB = readRegister(KEYPAD_TIME_LSB);

  return (MSB << 8) | LSB;
}

//Returns a string of the firmware version number
String KEYPAD::getVersion()
{
  uint8_t Major = readRegister(KEYPAD_VERSION1);
  uint8_t Minor = readRegister(KEYPAD_VERSION2);

  return("v" + String(Major) + "." + String(Minor));
}

//Change the I2C address of this address to newAddress
void KEYPAD::setI2CAddress(uint8_t newAddress)
{
  if (8 <= newAddress && newAddress <= 119)
  {
    writeRegister(KEYPAD_CHANGE_ADDRESS, newAddress);
    delay(100); // allow time for slave device to write new address to EEPROM

    //Once the address is changed, we need to change it in the library
    _deviceAddress = newAddress;
  }
}

//Reads from a given location from the Keypad
uint8_t KEYPAD::readRegister(uint8_t addr)
{
  TinyWireM.beginTransmission(_deviceAddress);
  TinyWireM.write(addr);

  if (TinyWireM.endTransmission() == 0) {

    TinyWireM.requestFrom(_deviceAddress, 1);
    return TinyWireM.available() ? TinyWireM.read() : 0;

  } else {

    return 0; //Device failed to ack
  }
}

//Write a byte value to a spot in the Keypad
boolean KEYPAD::writeRegister(uint8_t addr, uint8_t val)
{
  TinyWireM.beginTransmission(_deviceAddress);

  TinyWireM.write(addr);
  TinyWireM.write(val);

  return TinyWireM.endTransmission() == 0;
}

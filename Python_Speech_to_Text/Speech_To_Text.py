# Python program to translate
# speech to text and text to speech

import speech_recognition as sr
import pyttsx3
import time
import serial

def send_string_char_by_char(port, baud_rate, string_to_send, delay=0.1):
    """
    Send a string character by character to an STM32 board via UART
    
    Args:
        port (str): Serial port name (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux)
        baud_rate (int): Baud rate for the serial connection
        string_to_send (str): The string to send character by character
        delay (float): Delay between characters in seconds (default: 0.1)
    """
    try:
        # Open serial connection
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to {port} at {baud_rate} baud")
        
        # Send each character with a delay
        for char in string_to_send:
            ser.write(char.encode('utf-8'))
            print(f"Sent: '{char}'")
            time.sleep(delay)
        
        print(f"\nAll {len(string_to_send)} characters sent successfully!")
        
        # Close the connection
        ser.close()
        print("Serial connection closed")
        return True
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        return False

def SpeakText(command):
    """
    Function to convert text to speech
    """
    # Initialize the engine
    engine = pyttsx3.init()
    engine.say(command) 
    engine.runAndWait()

# Serial port configuration
PORT = "COM3"  # Change this to your serial port
BAUD_RATE = 115200  # Match this with your STM32 UART configuration
CHAR_DELAY = 0.1  # Delay between characters in seconds

# Initialize the recognizer 
r = sr.Recognizer() 

# Initialize Enable Button
enable = False

# Main program loop
print("Speech to UART program started")
print(f"UART Configuration: Port={PORT}, Baud Rate={BAUD_RATE}, Character Delay={CHAR_DELAY}s")
print("Press Ctrl+C to activate voice recognition")

while True:
    try:
        print("Awaiting voice input. Press Ctrl+C to input voice.")
        for i in range(1000):
            time.sleep(1)
    except KeyboardInterrupt:
        enable = True
        print("\nVoice recognition activated! Speak now...")

    while enable:    
        # Exception handling to handle exceptions at runtime
        try:
            # Use the microphone as source for input
            with sr.Microphone() as source2:
                
                # Wait for a second to let the recognizer
                # adjust the energy threshold based on
                # the surrounding noise level 
                r.adjust_for_ambient_noise(source2, duration=0.2)
                
                # Listens for the user's input 
                audio2 = r.listen(source2)
                
                # Using google to recognize audio
                MyText = r.recognize_google(audio2)
                MyText = MyText.Upper()
                
                # Check if "stop" is in the recognized text
                if "stop" in MyText:
                    print("Stop command detected. Deactivating voice recognition.")
                    enable = False
                    MyText = MyText.replace(" stop", "")
                print("Did you say: " + MyText)
                SpeakText(MyText)
                
                # Send the recognized text to STM32 via UART
                print("\nSending text to STM32 via UART...")
                send_string_char_by_char(PORT, BAUD_RATE, MyText, CHAR_DELAY)
                
                
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service:", e)
            enable = False
            
        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio")







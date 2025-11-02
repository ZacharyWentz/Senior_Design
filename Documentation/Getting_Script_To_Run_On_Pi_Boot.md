# Run a Python Script On Raspberry Pi Startup

1. Create a Python script to run on Raspberry Pi startup
    - Note where it is located on your Pi
2. Enter the following command in the terminal: <br>
    ```sudo nano /etc/sytemd/weedscout.service```
    
    - Change 'weedscout' to whatever you want. You do need a .service ending, though.

3. Enter the following script after running the nano command:
    ```
    [Unit]
    Description=WeedScout Script
    After=multi-user.target

    [Service]
    ExecStart=/usr/bin/python3 -u /home/pi/test.py
    StandardOutput=file:/home/pi/WeedScout_log.txt
    StandardError=file:/home/pi/WeedScout_error.txt
    Restart=always
    RestartSec=5

    [Install]
    WantedBy=multi-user.target
    ```

    - Change the Description to whatever you want
    - After=multi-user.target means our Python script will run once control is given to the user (from the kernel)
    - The second part of ExecStart is where we specify what Python script we want to run. /home/pi/test.py was the location of our Python script to run (adjust it to yours as needed)
    - StandardOutput and StandardError let you specify locations where data will be logged for normal logging or errors while your Python script executes. 
        - You can use "file:" or "append:" (we use "file:")
            - "file:" will overwrite anything at the specified location each time the Pi boots.
            - "append:" will not overwrite whatever is at the specified location but rather will append to the end of the file.
        - After "file:" or "append:", you can specify the path to a .txt file where logs or errors will be written to
    - Restart=always means that if the process exits or is killed, the service is restarted.
        - You can use Restart=no if you don't want to restart the service.
    - RestartSec=5 means the system will wait 5 seconds before attempting to restart the service that has stopped or been killed.
        - If you specify Restart=no, then you don't have to have this field

4. Hit Ctrl+O and then Enter to save the file. Then hit Crtl+X and then Enter to exit the file.
5. To tell systemd to recognize our service, enter the following in the terminal: <br>
    ```sudo systemctl daemon-reload```
    - Note: You will need to enter this command every time you change the .service file.
6. To tell systemd that we want our service to start on boot, enter the following in the terminal: <br>
    ```sudo systemctl enable /etc/systemd/weedscout.service```
    - Adjust the name 'weedscout' based on whatever you called your .service file in step #2.
7. To test your script runs on boot, issue the following command to restart your Pi: <br>
    ```sudo reboot```
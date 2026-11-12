
#ifndef HTMLFILE_H
#define HTMLFILE_H

#define HTTP_RESPONSE_BODY_TEMPLATE \
"<!DOCTYPE html>\n" \
"<html lang=\"en\">\n" \
"<head>\n" \
"    <meta charset=\"UTF-8\">\n" \
"    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n" \
"    <title>Motor Controller</title>\n" \
"    <style>\n" \
"        * { box-sizing: border-box; }\n" \
"        body {\n" \
"            margin: 0;\n" \
"            padding: 20px;\n" \
"            background: #17191d;\n" \
"            color: #f1f1f1;\n" \
"            font-family: Arial, sans-serif;\n" \
"        }\n" \
"        .container {\n" \
"            max-width: 480px;\n" \
"            margin: 30px auto;\n" \
"            padding: 25px;\n" \
"            background: #22252b;\n" \
"            border-radius: 14px;\n" \
"        }\n" \
"        h1 { text-align: center; margin-top: 0; }\n" \
"        .status {\n" \
"            text-align: center;\n" \
"            color: #aaa;\n" \
"            margin-bottom: 25px;\n" \
"        }\n" \
"        .status span { color: #4ade80; font-weight: bold; }\n" \
"        .section { margin: 25px 0; }\n" \
"        .section-title {\n" \
"            display: block;\n" \
"            margin-bottom: 12px;\n" \
"            font-weight: bold;\n" \
"        }\n" \
"        .rpm-display {\n" \
"            text-align: center;\n" \
"            font-size: 38px;\n" \
"            font-weight: bold;\n" \
"            margin: 15px 0;\n" \
"        }\n" \
"        input[type=range] {\n" \
"            width: 100%;\n" \
"            accent-color: #3b82f6;\n" \
"        }\n" \
"        .range-labels {\n" \
"            display: flex;\n" \
"            justify-content: space-between;\n" \
"            color: #aaa;\n" \
"            font-size: 12px;\n" \
"        }\n" \
"        button {\n" \
"            width: 100%;\n" \
"            padding: 14px;\n" \
"            margin-top: 10px;\n" \
"            border: none;\n" \
"            border-radius: 8px;\n" \
"            font-size: 16px;\n" \
"            font-weight: bold;\n" \
"            cursor: pointer;\n" \
"            color: white;\n" \
"        }\n" \
"        button:active { opacity: 0.75; }\n" \
"        button:disabled { opacity: 0.5; cursor: not-allowed; }\n" \
"        .send { background: #2563eb; }\n" \
"        .stop { background: #dc2626; }\n" \
"        .reverse { background: #6b7280; }\n" \
"        .reverse.active { background: #d97706; }\n" \
"        .source { background: #15803d; }\n" \
"        .source.pot { background: #7c3aed; }\n" \
"        .direction {\n" \
"            text-align: center;\n" \
"            color: #aaa;\n" \
"            margin-top: 10px;\n" \
"        }\n" \
"        .message {\n" \
"            text-align: center;\n" \
"            color: #aaa;\n" \
"            min-height: 20px;\n" \
"            margin-top: 18px;\n" \
"            font-size: 13px;\n" \
"        }\n" \
"    </style>\n" \
"</head>\n" \
"<body>\n" \
"    <div class=\"container\">\n" \
"        <h1>Motor Controller</h1>\n" \
"        <div class=\"status\">Connection: <span id=\"connection\">Ready</span></div>\n" \
"\n" \
"        <div class=\"section\">\n" \
"            <span class=\"section-title\">Motor speed</span>\n" \
"            <div class=\"rpm-display\"><span id=\"rpmValue\">3000</span> RPM</div>\n" \
"            <input type=\"range\" id=\"rpmSlider\" min=\"300\" max=\"6000\" step=\"10\" value=\"3000\">\n" \
"            <div class=\"range-labels\">\n" \
"                <span>300 RPM</span>\n" \
"                <span>6000 RPM</span>\n" \
"            </div>\n" \
"        </div>\n" \
"\n" \
"        <div class=\"section\">\n" \
"            <span class=\"section-title\">Direction</span>\n" \
"            <button class=\"reverse\" id=\"reverseButton\" onclick=\"toggleDirection()\">Reverse: OFF</button>\n" \
"            <div class=\"direction\" id=\"directionText\">Forward direction</div>\n" \
"        </div>\n" \
"\n" \
"        <div class=\"section\">\n" \
"            <button class=\"send\" onclick=\"sendMotorCommand()\">Send Speed</button>\n" \
"            <button class=\"stop\" onclick=\"stopMotor()\">STOP MOTOR</button>\n" \
"        </div>\n" \
"\n" \
"        <div class=\"section\">\n" \
"            <span class=\"section-title\">Control source</span>\n" \
"            <button class=\"source\" id=\"sourceButton\" onclick=\"toggleSource()\">Switch to Potentiometer</button>\n" \
"            <div class=\"direction\" id=\"sourceText\">Currently using webpage control</div>\n" \
"        </div>\n" \
"\n" \
"        <div class=\"message\" id=\"message\">Ready</div>\n" \
"    </div>\n" \
"\n" \
"    <script>\n" \
"        let reverse = false;\n" \
"        let webpageControl = true;\n" \
"\n" \
"        const slider = document.getElementById('rpmSlider');\n" \
"        const rpmValue = document.getElementById('rpmValue');\n" \
"        const connection = document.getElementById('connection');\n" \
"        const message = document.getElementById('message');\n" \
"\n" \
"        slider.addEventListener('input', function() {\n" \
"            rpmValue.textContent = slider.value;\n" \
"        });\n" \
"\n" \
"        async function sendRequest(parameters) {\n" \
"            connection.textContent = 'Sending...';\n" \
"            try {\n" \
"                const response = await fetch('/?' + new URLSearchParams(parameters), {\n" \
"                    method: 'GET',\n" \
"                    cache: 'no-store'\n" \
"                });\n" \
"                if (!response.ok) throw new Error('HTTP ' + response.status);\n" \
"                connection.textContent = 'Connected';\n" \
"                return true;\n" \
"            } catch (error) {\n" \
"                connection.textContent = 'Error';\n" \
"                message.textContent = 'Could not contact the Pico W';\n" \
"                return false;\n" \
"            }\n" \
"        }\n" \
"\n" \
"        function toggleDirection() {\n" \
"            reverse = !reverse;\n" \
"            const button = document.getElementById('reverseButton');\n" \
"            button.textContent = reverse ? 'Reverse: ON' : 'Reverse: OFF';\n" \
"            button.classList.toggle('active', reverse);\n" \
"            document.getElementById('directionText').textContent = reverse ? 'Reverse direction' : 'Forward direction';\n" \
"        }\n" \
"\n" \
"        async function sendMotorCommand() {\n" \
"            if (!webpageControl) {\n" \
"                message.textContent = 'Switch to webpage control first';\n" \
"                return;\n" \
"            }\n" \
"            let rpm = parseInt(slider.value, 10);\n" \
"            if (reverse) rpm = -rpm;\n" \
"            const success = await sendRequest({ motor: '1', rpm: String(rpm) });\n" \
"            if (success) message.textContent = 'Speed command sent: ' + rpm + ' RPM';\n" \
"        }\n" \
"\n" \
"        async function stopMotor() {\n" \
"            const success = await sendRequest({ stop: '1' });\n" \
"            if (success) message.textContent = 'Stop command sent';\n" \
"        }\n" \
"\n" \
"        async function toggleSource() {\n" \
"            const newSource = webpageControl ? 'pot' : 'page';\n" \
"            const success = await sendRequest({ source: newSource });\n" \
"            if (!success) return;\n" \
"            webpageControl = !webpageControl;\n" \
"            const button = document.getElementById('sourceButton');\n" \
"            const sourceText = document.getElementById('sourceText');\n" \
"            button.textContent = webpageControl ? 'Switch to Potentiometer' : 'Switch to Webpage';\n" \
"            button.classList.toggle('pot', !webpageControl);\n" \
"            sourceText.textContent = webpageControl ? 'Currently using webpage control' : 'Currently using potentiometer control';\n" \
"            message.textContent = 'Control source changed';\n" \
"        }\n" \
"        async function sendDeviceTime() {\n" \
"            const now = new Date();\n" \
"\n" \
"            const success = await sendRequest({\n" \
"                set_time: '1',\n" \
"                hour: String(now.getHours()),\n" \
"                minute: String(now.getMinutes()),\n" \
"                second: String(now.getSeconds()),\n" \
"                day: String(now.getDate()),\n" \
"                month: String(now.getMonth() + 1),\n" \
"                year: String(now.getFullYear())\n" \
"            });\n" \
"\n" \
"            if (success) {\n" \
"                message.textContent = 'Device date and time synchronised';\n" \
"            }\n" \
"        }\n" \
"\n" \
"        sendDeviceTime();\n" \
    "</script>\n" \
"</body>\n" \
"</html>\n"

#endif

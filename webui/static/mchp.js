// Copyright © 2002-2016 Microchip Technology Inc.  All rights reserved.
// See Microchip TCP/IP Stack documentation for license information.

// Determines when a request is considered "timed out".
var timeOutMS = 5000; // ms

// Stores a queue of AJAX events to process.
var ajaxList = new Array();

/**
 * Stores the number of dots current being shown in the rescan button.
 */
var scanDots = 0;

/**
 * Current iteration of the bss info read from Wi-Fi module.
 */
var currBss = 0;

var baseURL = ""; // leave blank for local webserver

/**
 * Whether to destroy or build other networks table.
 */
var otherNetworkExpanded = 1;

function bootWebsite()
{
   var urlPath = window.location.pathname;
   var pageName = urlPath.substring(urlPath.lastIndexOf('/') + 1);
   if (pageName == '' || pageName.startsWith('index')) {
        newAJAXCommand(baseURL+"ssids.cgi", updateFoundNetworks, false);
        newAJAXCommand(baseURL+"settings.cgi", getSettings, false);  
    }
    else if (pageName.startsWith('info'))
    {
        newAJAXCommand(baseURL+"status.cgi", getStatus, false);
    }
}


// Initiates a new AJAX command
//  url: the url to access
//  container: the document ID to fill, or a function to call with response XML (optional)
//  repeat: true to repeat this call indefinitely (optional)
//  data: an URL encoded string to be submitted as POST data (optional)
function newAJAXCommand(url, container, repeat, data)
{
    // Set up our object
    var newAjax = new Object();
    var theTimer = new Date();
    newAjax.url = url;
    newAjax.container = container;
    newAjax.repeat = repeat;
    newAjax.ajaxReq = null;

    // Create and send the request
    if (window.XMLHttpRequest) {
        newAjax.ajaxReq = new XMLHttpRequest();
        newAjax.ajaxReq.open((data==null)?"GET":"POST", newAjax.url, true);
        newAjax.ajaxReq.send(data);
    // If we're using IE6 style (maybe 5.5 compatible too)
    } else if (window.ActiveXObject) {
        newAjax.ajaxReq = new ActiveXObject("Microsoft.XMLHTTP");
        if (newAjax.ajaxReq) {
            newAjax.ajaxReq.open((data==null)?"GET":"POST", newAjax.url, true);
            newAjax.ajaxReq.send(data);
        }
    }

    newAjax.lastCalled = theTimer.getTime();

    // Store in our array
    ajaxList.push(newAjax);
}

// Loops over all pending AJAX events to determine if any action is required
function pollAJAX() {
    var curAjax = new Object();
    var theTimer = new Date();
    var elapsed;

    // Read off the ajaxList objects one by one
    for(i = ajaxList.length; i > 0; i--)
    {
        curAjax = ajaxList.shift();
        if(!curAjax)
            continue;
        elapsed = theTimer.getTime() - curAjax.lastCalled;

        // If we succeeded
        if (curAjax.ajaxReq.readyState == 4 && curAjax.ajaxReq.status == 200) {
            // If it has a container, write the result
            if (typeof(curAjax.container) == 'function') {
                curAjax.container(curAjax.ajaxReq.responseText);
            } else if (typeof(curAjax.container) == 'string') {
                document.getElementById(curAjax.container).innerHTML = curAjax.ajaxReq.responseText;
            } // (otherwise do nothing for null values)

            curAjax.ajaxReq.abort();
            curAjax.ajaxReq = null;

            // If it's a repeatable request, then do so
            if (curAjax.repeat)
                newAJAXCommand(curAjax.url, curAjax.container, curAjax.repeat);
            continue;
        }

        // If we've waited over 1 second, then we timed out
        if (elapsed > timeOutMS) {
            // Invoke the user function with null input
            if(typeof(curAjax.container) == 'function') {
                curAjax.container(null);
            } else {
                // Alert the user
                alert("Command failed.\nConnection to TTN Gateway was lost. Make sure you are connected to the same network.");
            }

            curAjax.ajaxReq.abort();
            curAjax.ajaxReq = null;

            // If it's a repeatable request, then do so
            if(curAjax.repeat)
                newAJAXCommand(curAjax.url, curAjax.container, curAjax.repeat);
            continue;
        }

        // Otherwise, just keep waiting
        ajaxList.push(curAjax);
    }

    // Call ourselves again in 10ms
    setTimeout("pollAJAX()",10);
}

// Parses the xmlResponse returned by an XMLHTTPRequest object
//  xmlData: the xmlData returned
//  field: the field to search for
function getXMLValue(xmlData, field, child)
{
    child = typeof(child) == "undefined" ? 0 : child;

    try {
        if(xmlData.getElementsByTagName(field)[child].firstChild.nodeValue)
            return xmlData.getElementsByTagName(field)[child].firstChild.nodeValue;
        else
            return null;
    } catch(err) { return null; }
}

function getStrength(str)
{
    if (str == 5)
        str = 4;

    return "images/bar-" + str + ".png";
}

function removeCheckmark()
{
    var table = document.getElementById('scanTable').getElementsByTagName("tbody")[0];
    var rows = table.rows;

    for (i = 1; i < rows.length; i++) {
        rows[i].lastChild.removeChild(rows[i].lastChild.lastChild);
        rows[i].lastChild.appendChild(document.createTextNode(""));
    }
}

function getSecurity(sec)
{
    var security;

    switch (sec)
    {
    case 0:
        security = "no";
        break;
    case 1:
        security = "wep";
        break;
    case 5: // WPA-PSK Security
        security = "wpa1";
        break;
    case 9: // WPA2-PSK Security
    case 13: // WPA-PSK/WPA2-PSK Auto Security, then use WPA2-PSK by default
        security = "wpa2";
        break;
    default:
        // Impossible to get here!
        break;
    }

    return security;
}

function securityKeyPrompt(secCode)
{
    var key;

    var wep64RE = new RegExp("^[0-9a-fA-F]{10}$"); // Check for hex key, 10 digits long
    var wep64REA = new RegExp("^.{5}$"); // Check for ASCII key, 5 digits long
    var wep128RE = new RegExp("^[0-9a-fA-F]{26}$"); // Check for hex key, 26 digits long
    var wep128REA = new RegExp("^.{13}$"); // Check for ASCII key, 13 digits long
    var wpaREA = new RegExp("^.{8,63}$"); // Check for ASCII passphrase, 8-63 characters long

    var errMsg;
    var keyInvalid = 0;
    if (secCode == 1) {
        do {
            msg = "Please enter your WEP key";
            if (keyInvalid) {
                msg = errMsg + "\r\n" + msg;
            }
            key = prompt(msg);
            if (key == null) {
                // User hit cancel, so need to go back to main screen and not submit
                // remove check mark also
                return "__UFU__";
            } else if (key == "") {
                // User hit enter only, so modify error message to notify user
                keyInvalid = 1;
                errMsg = "No key entered!\r\n";
            } else {
                // Key could be valid, check it
                if ((wep64RE.test(key) == false) && (wep64REA.test(key) == false) &&
                    (wep128RE.test(key) == false) && (wep128REA.test(key) == false)) {
                    keyInvalid = 1;
                    errMsg = "WEP key is not the correct length!\r\n";
                    errMsg += "Keys must be either:\r\n";
                    errMsg += "(a) 10 or 26 hexadecimal digits or\r\n";
                    errMsg += "(b) 5 or 13 ASCII characters\r\n";
                } else {
                    if ((key.length == 5) || (key.length == 10)) {
                        document.getElementById("sec").value += "40";
                    } else if ((key.length == 13 || (key.length == 26))) {
                        document.getElementById("sec").value += "104";
                    }
                    keyInvalid = 0;
                }
            }
        } while (keyInvalid);
    } else if (secCode > 1) {
        do {
            msg = "Please enter your passphrase";
            if (keyInvalid) {
                msg = errMsg + "\r\n" + msg;
            }
            key = prompt(msg);
            if (key == null) {
                // User hit cancel, so need to go back to main screen and not submit
                // remove check mark also
                return "__UFU__";
            } else if (key == "") {
                // User hit enter only, so modify error message to notify user
                keyInvalid = 1;
                errMsg = "No passphrase entered!\r\n";
            } else {
                // Passphrase could be valid, check it
                if (wpaREA.test(key) == false) {
                    keyInvalid = 1;
                    errMsg = "WPA passphrase does not have the correct length!\r\n";
                    errMsg += "Passphrase must be 8-63 characters long.\r\n";
                } else {
                    keyInvalid = 0;
                }
            }
        } while (keyInvalid);
    }
    return key;
}

// ======================================
// = Connect to the Passed Network SSID =
// ======================================
function switchNetwork(id)
{
    if (document.getElementById("rescan").disabled == true) {
        // Don't allow users to click any of the network names
        // if we are currently doing a scan
        return;
    }

    var networkParam = new Array();
    networkParam = id.split("\020", 3);

    var name = networkParam[0];
    var secCode = networkParam[1];
    var wlanCode = networkParam[2];

    if (name == "")
    {
        alert('SSID cannot be left blank!');
        return;
    }
    else if (wlanCode == "undefined")
    {
        alert('Please choose either Ad-Hoc or Infrastructure');
        return;
    }

    removeCheckmark();
    var row = document.getElementById(id);
    if (row != null) {
        // Null means that the row doesn't exist.
        // Only happens when we are entering a SSID manually
        // from the Other Network... dropdown.
        row.lastChild.removeChild(row.lastChild.lastChild);

        var checkImg = document.createElement("img");
        checkImg.src = "images/checkmark.png";
        row.lastChild.appendChild(checkImg);
    }

    document.getElementById("wlan").value = (wlanCode == 1) ? "infra" : "adhoc";
    document.getElementById("ssid").value = name;
    document.getElementById("sec").value = getSecurity(parseInt(secCode));

    var key;
    if (parseInt(secCode)) {
        key = securityKeyPrompt(secCode);
        document.getElementById("key").value = key;
    }

    if (key == "__UFU__") {
        // User hit cancel on the prompt box for the network
        // so assume they didn't want to set the network
        removeCheckmark();
        document.getElementById("key").value = null;
    } else {
        document.kickoff.submit();
    }
}

// =============================
// = Join the Selected Network =
// =============================
function joinNetwork()
{
    // Copy elements from hidden form and submit them through
    // the normal form
    var id;

    id = document.getElementById('ssid1').value + "\020";
    id += document.getElementById('sec1').value + "\020";

    var wlanVal;
    var wlanRadio = document.getElementsByName('wlan1');
    for (i = 0; i < wlanRadio.length; i++) {
        if (wlanRadio[i].checked) {
            wlanVal = 2 - i;
            break;
        }
    }

    id += wlanVal;

    // Now switch to the network
    switchNetwork(id);
}

// =====================================
// = User Entered Network Routine      =
// = configure.html "Other Network..." =
// =====================================
function userSelectNetwork() {
    // Add rows for adhoc/infra selection, text input of ssid and
    // selector box for security type.
    // If security type other than none is chosen, append additional
    // text input field for key/passphrase.

    otherNetworkExpanded = (otherNetworkExpanded == 1) ? 0 : 1;

    if (otherNetworkExpanded)
    {
        // Need to destroy table back to just button
        var table = document.getElementById('scanTable').getElementsByTagName('tfoot')[0];
        var rows = table.rows;

        while (rows.length - 1) // length = 1 -> stop
            table.deleteRow(rows.length - 1);
    } else {
        var tfoot = document.getElementById('scanTable').getElementsByTagName("tfoot")[0];

        var row1 = document.createElement("tr");
        row1.setAttribute('style', 'width:9em');

        var data1 = document.createElement("td");
        data1.setAttribute('colspan', 3);
        data1.appendChild(document.createTextNode('Ad-Hoc'));

        var data2 = document.createElement("td");
        var adhocInput = document.createElement("input");
        adhocInput.setAttribute('type', 'radio');
        adhocInput.setAttribute('name', 'wlan1');
        adhocInput.setAttribute('value', '2');
        adhocInput.setAttribute('onclick', 'adhocSel();');
        data2.appendChild(adhocInput);

        row1.appendChild(data1);
        row1.appendChild(data2);

        var row2 = document.createElement("tr");
        row2.setAttribute('style', 'width:9em');

        var data3 = document.createElement("td");
        data3.setAttribute('colspan', 3);
        data3.appendChild(document.createTextNode('Infrastructure'));

        var data4 = document.createElement("td");
        var infraInput = document.createElement("input");
        infraInput.setAttribute('type', 'radio');
        infraInput.setAttribute('name', 'wlan1');
        infraInput.setAttribute('value', '1');
        infraInput.setAttribute('onclick', 'infraSel();');
        data4.appendChild(infraInput);

        row2.appendChild(data3);
        row2.appendChild(data4);

        var row3 = document.createElement("tr");
        row3.setAttribute('style', 'width:9em');
        var data5 = document.createElement("td");
        data5.setAttribute('colspan', '4');
        data5.appendChild(document.createTextNode("Network Name"));
        row3.appendChild(data5);

        var row4 = document.createElement("tr");
        row4.setAttribute('style', 'width:9em');
        var data6 = document.createElement("td");
        data6.setAttribute('colspan', '4');
        var ssidInput = document.createElement("input");
        ssidInput.setAttribute('type', 'text');
        ssidInput.setAttribute('id', 'ssid1');
        ssidInput.setAttribute('name', 'ssid1');
        ssidInput.setAttribute('maxlength', '32');
        data6.appendChild(ssidInput);

        row4.appendChild(data6);

        var row5 = document.createElement("tr");
        row5.setAttribute('style', 'width:9em');
        var data7 = document.createElement("td");
        data7.setAttribute('colspan', '4');
        var secSel = document.createElement("select");
        secSel.setAttribute('name', 'sec1');
        secSel.setAttribute('id', 'sec1');
        secSel.options[0] = new Option('None', '0');
        secSel.options[1] = new Option('WEP', '1');
        secSel.options[2] = new Option('WPA-PSK', '5');
        secSel.options[3] = new Option('WPA2-PSK', '9');
        data7.appendChild(secSel);
        row5.appendChild(data7);

        var row6 = document.createElement("tr");
        row6.setAttribute('style', 'width:9em');
        var data8 = document.createElement("td");
        data8.setAttribute('colspan', '3');
        var joinButton = document.createElement("input");
        joinButton.setAttribute('id', 'joinButton');
        joinButton.setAttribute('type', 'button');
        joinButton.setAttribute('value', 'Join');
        joinButton.setAttribute('onclick', 'joinNetwork();');
        data8.appendChild(joinButton);
        row6.appendChild(data8);

        tfoot.appendChild(row1);
        tfoot.appendChild(row2);
        tfoot.appendChild(row3);
        tfoot.appendChild(row4);
        tfoot.appendChild(row5);
        tfoot.appendChild(row6);
    }
}

function adhocSel()
{
    if (document.getElementById('sec1').options.length == 4) {
        document.getElementById('sec1').remove(3);
        document.getElementById('sec1').remove(2);
    }
}

function infraSel()
{
    if (document.getElementById('sec1').options.length == 2) {
        document.getElementById('sec1').options[2] = new Option('WPA-PSK', '5');
        document.getElementById('sec1').options[3] = new Option('WPA2-PSK', '9');
    }
}

// ===============================================
// = Add Wi-Fi Network Information to Scan Table =
// ===============================================
function addScanRow(ssid, sec, str, wlan)
{
    var tbody = document.getElementById('scanTable').getElementsByTagName("tbody")[0];
    var row = document.createElement("tr");

    var blankImg = document.createElement("img");
    blankImg.src = "images/blank.png";

    row.setAttribute('id', ssid + "\020" + sec + "\020" + wlan);
    row.setAttribute('onmouseover', "this.style.cursor='pointer'");
    row.setAttribute('onclick', 'switchNetwork(id);');

    var data1 = document.createElement("td");
    data1.setAttribute('style', 'width:10em');
    data1.appendChild(document.createTextNode(ssid));

    var data2 = document.createElement("td");
    var secImg = document.createElement("img");
    secImg.src = "images/lock.png";
    if (sec > 0) {
        data2.appendChild(secImg);
    } else {
        data2.appendChild(blankImg);
    }

    var data3 = document.createElement("td");
    var pwrImg = document.createElement("img");
    pwrImg.src = getStrength(str);
    data3.appendChild(pwrImg);

    var data4 = document.createElement("td");
    data4.appendChild(blankImg);

    row.appendChild(data1);
    row.appendChild(data2);
    row.appendChild(data3);
    row.appendChild(data4);

    tbody.appendChild(row);
}

// ===========================
// = Wi-Fi delete scan table =
// ===========================


function deleteScanTable()
{
    var x = document.getElementById("ssid");

    var i=x.length;
    for(;i>0;i--)
    {
        x.remove(0);
    }
}

function printButtonName()
{
    var textLabel = "Scanning";

    for (i = 0; i < scanDots % 4; i++) {
        textLabel += ".";
    }

    scanDots++;

    document.getElementById("rescan").value = textLabel;
}

function validateSecurityKey(secCode,wifiKey,errorBox)
{
    var key;

    var wep64RE = new RegExp("^[0-9a-fA-F]{10}$"); // Check for hex key, 10 digits long
    var wep64REA = new RegExp("^.{5}$"); // Check for ASCII key, 5 digits long
    var wep128RE = new RegExp("^[0-9a-fA-F]{26}$"); // Check for hex key, 26 digits long
    var wep128REA = new RegExp("^.{13}$"); // Check for ASCII key, 13 digits long
    var wpaREA = new RegExp("^.{8,63}$"); // Check for ASCII passphrase, 8-63 characters long

    var errMsg;
    if (secCode == "wep") {
        key = wifiKey;
        if (key == null) {
            // User hit cancel, so need to go back to main screen and not submit
            // remove check mark also
            errorBox.innerHTML = "No key entered!\r\n";
            return false;
        } else if (key == "") {
            // User hit enter only, so modify error message to notify user
            errorBox.innerHTML = "No key entered!\r\n";
            return false;
        } else {
            // Key could be valid, check it
            if ((wep64RE.test(key) == false) && (wep64REA.test(key) == false) &&
                (wep128RE.test(key) == false) && (wep128REA.test(key) == false)) {
                
                errorBox.innerHTML = "WEP key is not the correct length!\r\n";
                errorBox.innerHTML += "Keys must be either:\r\n";
                errorBox.innerHTML += "(a) 10 or 26 hexadecimal digits or\r\n";
                errorBox.innerHTML += "(b) 5 or 13 ASCII characters\r\n";
                return false;
            } else {
                if ((key.length == 5) || (key.length == 10)) {
                    document.getElementById("sec").value += "40";
                } else if ((key.length == 13 || (key.length == 26))) {
                    document.getElementById("sec").value += "104";
                }
                keyInvalid = 0;
            }
        }
    } else if (secCode !="no") {
        key = wifiKey;
        if (key == null) {
            errorBox.innerHTML = "No passphrase entered!\r\n";
            return false;
        } else if (key == "") {
            errorBox.innerHTML = "No passphrase entered!\r\n";
            return false;
        } else {
            // Passphrase could be valid, check it
            if (wpaREA.test(key) == false) {
                errorBox.innerHTML = "WPA passphrase does not have the correct length!\r\n";
                errorBox.innerHTML += "Passphrase must be 8-63 characters long.\r\n";
                return false;
            } 
        }
    }
    else
    {
        return true;
    }
    return true;
}


function validateActivationForm()
{
    var t_gatewayid = document.forms["form"]["gwid"].value;
    if(t_gatewayid.length<6) {
        document.getElementById("gwiderror").style.display = "block";
        return false;
    }
    else document.getElementById("gwiderror").style.display = "none";


    var t_gatewaykey = document.getElementById("gwky").value;
    if(t_gatewaykey.length<=0)
    {
        document.getElementById("gwky").disabled =true;
    }


    var wifiSelect = document.getElementById("ssid");
    if(wifiSelect.selectedIndex > 0) // DO WIFI REJOIN
    {
        var pw = document.getElementById("key");
        var secfield = document.getElementById("sec");
        var errorfield= document.getElementById("keyerror");
        if(validateSecurityKey(secfield.value,pw.value,errorfield))
        {
            document.getElementById("keyerror").style.display = "none";
        }
        else
        {
            document.getElementById("keyerror").style.display = "block";
            return false;
        }
    }
    else // NO WIFI REJOIN
        {
            document.getElementById("key").disabled =true;
            document.getElementById("sec").disabled =true;
            document.getElementById("wlan").disabled =true;
            document.getElementById("ssid").disabled =true;
            wifiSelect.disabled = true;
        }

    return true;
}






function rescanNetwork()
{
    scanDots = 0;
   deleteScanTable();
   var x = document.getElementById("ssid");
   var option = document.createElement("option");
   option.text = "Scanning...";
   x.add(option);

   document.getElementById("scanbtn").style.visibility = "hidden";
   newAJAXCommand(baseURL+"scan.cgi?scan=1");
    setTimeout("newAJAXCommand(baseURL+\"ssids.cgi\", updateFoundNetworks, false)", 2000);

}

var currentNetwork = "";
var gatewayid = "";
var gatewaykey = "";
var activationURL = "";
var networks = [];

function updateFoundNetworks(xmlData)
{
    deleteScanTable();

    document.getElementById("scanbtn").style.visibility = "visible";
    var data = JSON.parse(xmlData.trim());
    var ssiddropdown = document.getElementById('ssid');
    var option = document.createElement("option");
    var i=0;
    option.text = "Found "+data.APS.length+" networks.";
    ssiddropdown.add(option);
    for(i=0;i<data.APS.length;i++)
    {
        networks[i] = {};
        networks[i].name = data.APS[i].name;
        networks[i].sec = data.APS[i].sec;
        networks[i].type = data.APS[i].type;
        var opt = document.createElement("option");
        opt.text = data.APS[i].name;
        opt.setAttribute('value',data.APS[i].name);

        ssiddropdown.appendChild(opt);
    }   
    if(data.APS.length > 0) selectFoundNetwork(0);
}

function getSettings(xmlData)
{
    var data = JSON.parse(xmlData.trim());

    currentNetwork = data.ssid;
    gatewayid = data.gwid;
    gatewaykey = data.gwkey;
    activationURL = data.asrv;
    locked = data.locked;

    document.getElementById('gwid').value = gatewayid;
    if (locked) {
        // when locked we get only last x characters of the key
        document.getElementById('gwky').value = '[...]'+gatewaykey;
    } else {
        document.getElementById('gwky').value = gatewaykey;
    }
    document.getElementById('asrv').value = activationURL;
    document.getElementById('gwid').disabled = locked;
    document.getElementById('gwky').disabled = locked;
    document.getElementById('asrv').disabled = locked;
}

function selectFoundNetwork(id)
{
    if(id>networks.length) return;
    document.getElementById("sec").value = networks[id].sec;
    document.getElementById("wlan").value = networks[id].type;
}

function getStatus(xmlData)
{
    var data = JSON.parse(xmlData.trim());
    document.getElementById('hwversion').innerHTML = data.hwversion;
    document.getElementById('blversion').innerHTML = data.blversion;
    document.getElementById('fwversion').innerHTML = data.fwversion;    
    document.getElementById('uptime').innerHTML = data.uptime;
    document.getElementById('connected').innerHTML = data.connected;
    document.getElementById('interface').innerHTML = data.interface;
    document.getElementById('ssid').innerHTML = data.ssid;
    document.getElementById('activation_locked').innerHTML = data.activation_locked;
    document.getElementById('configured').innerHTML = data.configured;
    document.getElementById('region').innerHTML = data.region;
    document.getElementById('connbroker').innerHTML = data.connbroker;
    document.getElementById('gwcard').innerHTML = data.gwcard;
    document.getElementById('pup').innerHTML = data.pup;
    document.getElementById('pdown').innerHTML = data.pdown;
    document.getElementById('estor').innerHTML = data.estor;
    setTimeout("newAJAXCommand(baseURL+'status.cgi', getStatus, false)",2000);
}

// Kick off the AJAX Updater
setTimeout("pollAJAX()", 10);

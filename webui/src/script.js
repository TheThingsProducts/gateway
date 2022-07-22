const runSite = async () => {
  const $hwversion = document.querySelector("#hwversion");
  const $blversion = document.querySelector("#blversion");
  const $fwversion = document.querySelector("#fwversion");

  const $uptime = document.querySelector("#uptime");
  const $estor = document.querySelector("#estor");
  const $connected = document.querySelector("#connected");
  const $interface = document.querySelector("#interface");
  const $ssid = document.querySelector("#ssid");
  const $gwcard = document.querySelector("#gwcard");
  const $configured = document.querySelector("#configured");
  const $region = document.querySelector("#region");
  const $connbroker = document.querySelector("#connbroker");
  const $pup = document.querySelector("#pup");
  const $pdown = document.querySelector("#pdown");

  const setSpanValue = ($el, value) => {
    if (!$el) {
      return;
    }
    $el.innerText = value;
  };

  const loadStatus = async () => {
    fetch("status.cgi")
      .then((response) => response.json())
      .then((data) => {
        console.log("status", data);
        setSpanValue($hwversion, data.hwversion);
        setSpanValue($blversion, data.blversion);
        setSpanValue($fwversion, data.fwversion);
        setSpanValue($uptime, data.uptime);
        setSpanValue($estor, data.estor);
        setSpanValue($connected, data.connected);
        setSpanValue($interface, data.interface);
        setSpanValue($ssid, data.ssid);
        setSpanValue($gwcard, data.gwcard);
        setSpanValue($configured, data.configured);
        setSpanValue($region, data.region);
        setSpanValue($connbroker, data.connbroker);
        setSpanValue($pup, data.pup);
        setSpanValue($pdown, data.pdown);
      });
  };

  try {
    loadStatus();
  } catch (e) {
    alert(e);
  }
};

document.addEventListener("DOMContentLoaded", runSite);

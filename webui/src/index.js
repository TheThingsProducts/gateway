const runPage = async () => {
  const $form = document.querySelector("form");
  const $asrv = document.querySelector("#asrv");
  const $gwid = document.querySelector("#gwid");
  const $gwky = document.querySelector("#gwky");
  const $startScan = document.querySelector("#startScan");
  const $ssidList = document.querySelector("#ssidList");
  const $ssidListItem = $ssidList.querySelector("label").cloneNode(true);
  const $wlan = document.querySelector("#wlan");
  const $sec = document.querySelector("#sec");
  const $key = document.querySelector("#key");

  const setFieldErr = ($field, err) => {
    if (err) {
      $field.parentElement.classList.add("invalid");
      $field.parentElement.querySelector(".error").innerText = err;
    } else {
      $field.parentElement.classList.remove("invalid");
    }
  };

  const loadSettings = async () => {
    fetch("settings.cgi")
      .then((response) => response.json())
      .then((data) => {
        console.log("settings", data);
        $asrv.value = data.asrv;
        $asrv.disabled = data.locked;
        $gwid.value = data.gwid;
        $gwid.disabled = data.locked;
        $gwky.value = data.locked ? "[...]" + data.gwkey : data.gwkey;
        $gwky.disabled = data.locked;
      });
  };

  let loadingSSIDs = false;

  const loadSSIDs = async (e) => {
    if (e) {
      e.preventDefault();
    }
    if (loadingSSIDs) {
      return;
    }
    loadingSSIDs = true;
    $ssidList.innerHTML = null;
    const $classList = $startScan.querySelector("img").classList;
    $classList.add("spinning");
    fetch("scan.cgi?scan=1").then(() => {
      fetch("ssids.cgi")
        .then((response) => response.json())
        .then((data) => {
          console.log("ssids", data);
          data.APS.sort((a, b) => a.rssi < b.rssi).forEach((ap) => {
            const $ap = $ssidListItem.cloneNode(true);
            $ap.querySelector("input").value = ap.name;
            $ap.querySelector("input").addEventListener("change", (e) => {
              if (!e.target.checked) {
                return;
              }
              $wlan.value = ap.type;
              $sec.value = ap.sec;
              $key.disabled = ap.sec == "no";
              $key.value = ap.sec == "no" ? "" : $key.value;
            });
            $ap.querySelector(
              "span"
            ).innerText = `${ap.name} (${ap.sec} security)`;
            let bars = 1;
            if (ap.rssi > -30) {
              bars = 4;
            } else if (ap.rssi > -67) {
              bars = 3;
            } else if (ap.rssi > -70) {
              bars = 2;
            }
            $ap.querySelector("img").src = `images/bar-${bars}.svg`;
            $ssidList.appendChild($ap);
          });
          $classList.remove("spinning");
          loadingSSIDs = false;
        });
    });
  };

  $startScan.addEventListener("click", loadSSIDs);

  $form.onsubmit = (e) => {
    setFieldErr($asrv);
    setFieldErr($gwid);
    setFieldErr($gwky);
    setFieldErr($key);
    let err = false;
    if (!$asrv.disabled) {
      if (!$asrv.value.match(/https?:\/\//)) {
        err = true;
        setFieldErr($asrv, "must start with http:// or https://");
      }
    }
    if (!$gwid.disabled) {
      if ($gwid.value.length < 3 || $gwid.value.length > 36) {
        err = true;
        setFieldErr($gwid, "minimum length: 3, maximum length: 36");
      } else if (
        !$gwid.value.match(
          /^[a-z0-9](?:[-]?[a-z0-9]){2,}(@[a-z0-9](?:[-]?[a-z0-9]){2,})?$/
        )
      ) {
        err = true;
        setFieldErr(
          $gwid,
          "must only contain lowercase letters, numbers and dashes"
        );
      }
    }
    if (!$gwky.disabled) {
      if ($gwky.value.length == 0) {
        err = true;
        setFieldErr($gwky, "must be set");
      }
    }
    if ($wlan.value && $sec.value !== "no" && $key.value.length == 0) {
      err = true;
      setFieldErr(
        $key,
        "must be set on network with " + $sec.value + " security"
      );
    }
    return !err;
  };

  try {
    loadSettings();
    loadSSIDs();
  } catch (e) {
    alert(e);
  }
};

document.addEventListener("DOMContentLoaded", runPage);

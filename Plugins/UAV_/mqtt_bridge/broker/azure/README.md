# HiveMQ test broker on Azure — runbook

Test broker for the MAVLink-over-MQTT bridge. **Simulator and test data only**
until the programme decides where flight data may be hosted.

What it is: HiveMQ (`hivemq/hivemq4` image) on one Ubuntu VM in South Africa
North. TLS on 8883 only, Let's Encrypt certificate, File RBAC users with
per-topic permissions, allow-all removed, Control Center on localhost only.

Files in this folder:

| File | Goes to (on the VM) |
|---|---|
| `config.xml` | `~/hivemq/config.xml` → container `/opt/hivemq/conf/config.xml` |
| `rbac-extension-config.xml` | `~/hivemq/extensions/hivemq-file-rbac-extension/conf/config.xml` |
| `credentials.xml.template` | becomes `.../hivemq-file-rbac-extension/conf/credentials.xml` (step 6) |

Never commit: `credentials.xml`, `broker.env`, keystores (see `.gitignore`).

## Current deployment (built 2026-10-29)

| | |
|---|---|
| Subscription | Azure for Students (Taariq's university account), test data only |
| Resource group | `BVLOS-COMS` |
| VM | `TDC-Cloud`, Ubuntu 24.04, South Africa North |
| Firewall | `TDC-Cloud-nsg`: 8883 from Any; 22 from the office public IP only |
| Broker name | `hivemq-broker.southafricanorth.cloudapp.azure.com` |
| HiveMQ | 4.56.1, trial licence (25 connections), File RBAC 4.6.19 |
| Certificate | Let's Encrypt, expires **2027-01-27** |
| Verified | 2026-10-29 from the XBLink over LTE: `smoke-test` PASS; wrong password refused; `uav01-aircraft` refused on the smoke topic |

Passwords are not stored anywhere in the repo or on the VM in plain text; the
VM holds only hashes.

---

## A. Create the VM (Azure portal)

1. **Virtual machines → Create → Azure virtual machine.**
2. Basics:
   - Subscription: **Azure for Students**. Resource group: new, `denel-mqtt-rg`.
   - VM name: `denel-mqtt-broker`. Region: **(Africa) South Africa North**.
   - Availability options: No infrastructure redundancy required. Security type: Standard.
   - Image: **Ubuntu Server 24.04 LTS – x64 Gen2**.
   - Size: **B2ats_v2** (2 vCPU, 1 GiB) or **B1s**. Pick one marked *Free services eligible*.
   - Authentication: **SSH public key**. Username `azureuser`. Key source: **Use existing public key**, paste the contents of `%USERPROFILE%\.ssh\id_ed25519.pub`.
   - Public inbound ports: Allow selected → **SSH (22)**.
3. Disks: OS disk **Premium SSD, 64 GiB (P6)**. Tick *Delete with VM*.
4. Networking: defaults (new public IP, Basic NSG). Tick *Delete public IP and NIC when VM is deleted*.
5. **Review + create → Create.** Wait for *Deployment complete* → **Go to resource**.
6. Overview → **DNS name: Not configured** → set a DNS name label (e.g. `denel-mqtt-tc`) → Save.
   Your broker name is `<label>.southafricanorth.cloudapp.azure.com`.
7. Network settings → **Create port rule → Inbound**:
   - `8883`, TCP, source Any, name `allow-mqtts`.
   - `80`, TCP, source Any, name `temp-certbot` (deleted in step 9).
   - Edit the SSH rule: source **My IP address**.

---

## B. Set up the VM

From PowerShell on the PC:

```
ssh azureuser@<dns-name>
```

### 1. Packages and swap (1 GiB RAM needs swap for Java)

```
sudo apt update && sudo apt -y upgrade
sudo apt -y install docker.io certbot unzip
sudo fallocate -l 2G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
sudo docker pull hivemq/hivemq4
mkdir -p ~/hivemq/certs ~/hivemq/extensions
```

### 2. Copy the files (PowerShell on the PC)

```
cd C:\MissionPlanner-denel\Plugins\UAV_\mqtt_bridge\broker\azure
scp config.xml rbac-extension-config.xml credentials.xml.template azureuser@<dns-name>:~/hivemq/
```

### 3. Certificate (back on the VM)

```
DNS=<dns-name>
sudo certbot certonly --standalone -d $DNS --agree-tos -m <your-email> --no-eff-email
```

### 4. Keystore and env file

```
read -s -p "New keystore password: " KSP; echo; export KSP
printf '%s' "$KSP" | sudo openssl pkcs12 -export -name broker \
  -in /etc/letsencrypt/live/$DNS/fullchain.pem -inkey /etc/letsencrypt/live/$DNS/privkey.pem \
  -out ~/hivemq/certs/broker.p12 -passout stdin
sudo --preserve-env=KSP docker run --rm --user 0 -e KSP -v ~/hivemq/certs:/certs --entrypoint keytool hivemq/hivemq4 \
  -importkeystore -noprompt -srckeystore /certs/broker.p12 -srcstoretype PKCS12 -srcstorepass:env KSP \
  -destkeystore /certs/broker.jks -deststoretype JKS -deststorepass:env KSP -destkeypass:env KSP
sudo rm ~/hivemq/certs/broker.p12
(umask 077; printf 'KEYSTORE_PASSWORD=%s\n' "$KSP" > ~/hivemq/broker.env)
unset KSP
sudo chown -R 10000:10000 ~/hivemq/certs && sudo chmod 700 ~/hivemq/certs && sudo chmod 600 ~/hivemq/certs/broker.jks
```

### 5. File RBAC extension

```
cd ~/hivemq
wget https://github.com/hivemq/hivemq-file-rbac-extension/releases/download/4.6.19/hivemq-file-rbac-extension-4.6.19.zip
unzip hivemq-file-rbac-extension-4.6.19.zip -d extensions
ls extensions/hivemq-file-rbac-extension extensions/hivemq-file-rbac-extension/conf
cp rbac-extension-config.xml extensions/hivemq-file-rbac-extension/conf/config.xml
```

If the download 404s, get the zip from
<https://github.com/hivemq/hivemq-file-rbac-extension/releases/latest> and
use its version number in steps 5 and 6.

### 6. Users and passwords

One password per user. Store them in a password manager.

```
H() { read -s -p "Password for $1: " P </dev/tty; echo >&2
      sudo docker run --rm -v ~/hivemq/extensions/hivemq-file-rbac-extension:/ext:ro --entrypoint java hivemq/hivemq4 \
        -jar /ext/hivemq-file-rbac-extension-4.6.19.jar -q -i 100000 -p "$P"; unset P; }
A=$(H uav01-aircraft); G=$(H uav01-gcs); S=$(H smoke-test)
sed -e "s|__HASH_AIRCRAFT__|$A|" -e "s|__HASH_GCS__|$G|" -e "s|__HASH_SMOKE__|$S|" \
  credentials.xml.template > extensions/hivemq-file-rbac-extension/conf/credentials.xml
grep -c "__HASH" extensions/hivemq-file-rbac-extension/conf/credentials.xml
unset A G S
sudo chown -R 10000:10000 extensions
```

The `grep -c` must print `0`. Paste the `A=`, `G=` and `S=` lines one at a time.

### 7. Start the broker

```
sudo docker run -d --name hivemq --restart unless-stopped \
  --ulimit nofile=500000:500000 \
  -p 8883:8883 -p 127.0.0.1:8080:8080 \
  -e HIVEMQ_ALLOW_ALL_CLIENTS=false \
  -e JAVA_OPTS="-Xms256m -Xmx512m" \
  --env-file ~/hivemq/broker.env \
  -v ~/hivemq/config.xml:/opt/hivemq/conf/config.xml:ro \
  -v ~/hivemq/certs:/opt/hivemq/certs:ro \
  -v ~/hivemq/extensions/hivemq-file-rbac-extension:/opt/hivemq/extensions/hivemq-file-rbac-extension \
  hivemq/hivemq4
sudo docker logs -f hivemq
```

Expect in the log: a TLS listener started on port **8883**, *Disabling allow
all extension*, and the **File Role Based Access Control** extension started
successfully. No `1883`. Ctrl+C leaves the log; the broker keeps running.

Optional, Lab licence: add `-v ~/hivemq/license.lic:/opt/hivemq/license/license.lic:ro`.

### 8. Test from the XBLink

Positive (expect PASS):

```
cd /opt/denel-mqtt
PYTHONPATH=/opt/denel-mqtt/lib python3 -m mqtt_bridge.tools.mqtt_smoke --host <dns-name> --tls --username smoke-test
```

Negative (both must FAIL):

```
# wrong password -> "broker refused the connection"
PYTHONPATH=/opt/denel-mqtt/lib python3 -m mqtt_bridge.tools.mqtt_smoke --host <dns-name> --tls --username smoke-test
# aircraft user on the smoke topic -> "nothing routed" (not permitted)
PYTHONPATH=/opt/denel-mqtt/lib python3 -m mqtt_bridge.tools.mqtt_smoke --host <dns-name> --tls --username uav01-aircraft
```

### 9. Close port 80

Portal → VM → Network settings → delete `temp-certbot`.

---

## Operating it

- Control Center: `ssh -L 8080:localhost:8080 azureuser@<dns-name>`, then <http://localhost:8080>. Change the default login.
- Change users/permissions: `sudo nano ~/hivemq/extensions/hivemq-file-rbac-extension/conf/credentials.xml` (hash new passwords with `H` from step 6). Reloaded within 60 s; affects new connections only.
- Stop when idle (saves credit): portal → VM → **Stop**. Start again before testing. The DNS name stays the same.
- Certificate expires after 90 days. Renewal needs port 80 open briefly and the keystore rebuilt (steps 3–4), then `sudo docker restart hivemq`. To be automated later.

-- vim:et:sts=2:sw=2:si
local gps_alt_id
local beacon_pref_id
local beacon_hdop_id
local beacon_number_of_sat_id
local beacon_status_id
local beacon_frames_id
local gpg_gps_id
local gpg_date_id
local gpg_alt_id
local gpg_speed_id
local gpg_hdg_id

local last_lat
local last_lon
local dt
local got_gps = false
local got_dt = false

local status_to_desc = {
  [0] = "GPS Fix",
  [1] = "GPS Prec",
  [2] = "Pret au sol",
  [3] = "En vol",
  [4] = "No sensor"
}

local status_to_flag = {
  [0] = BLINK,
  [1] = BLINK,
  [2] = 0,
  [3] = 0,
  [4] = BLINK
}

local function lookForId(...)
  local args = {...}
  for _, v in ipairs(args) do
    item = getFieldInfo(v)
    if item ~= nil then
      return item.id
    end
  end
  return 0
end

local function init()
  beacon_pref_id =          lookForId("Pref", "5230")
  beacon_hdop_id =          lookForId("HDOP", "5220")
  beacon_number_of_sat_id = lookForId("NSat", "5210")
  beacon_status_id =        lookForId("BSta", "5200")
  beacon_frames_id =        lookForId("BFra", "5240")
  gps_gps_id =              lookForId("GPS")
  gps_alt_id =              lookForId("GAlt")
  gps_speed_id =            lookForId("GSpd")
  gps_hdg_id =              lookForId("Hdg")
  gps_date_id =             lookForId("Date")
end

local function background()
end

local function run(event)
  lcd.clear() 
  lcd.drawScreenTitle('LaBalise v0.1', 0, 0)
  local status
  if beacon_status_id ~= 0 then
    status = getValue(beacon_status_id)
    if status < 0 or status > 4 then
      status = 4
    end
  else
    status = 4
  end
  lcd.drawText(128, 1,   status_to_desc[status], SMLSIZE + RIGHT + status_to_flag[status])

  if gps_alt_id ~= 0 then
    lcd.drawText(1, 14, "Alt", SMLSIZE)
    lcd.drawText(72, 10, string.format("%dm", getValue(gps_alt_id)), DBLSIZE + RIGHT)
  end

  if beacon_pref_id ~= 0 then
    local flags = 0
    local gv7 = model.getGlobalVariable(7, 0)
    local gv8 = model.getGlobalVariable(8, 0)
    if getValue(beacon_pref_id) ~= (gv7 * 1000 + gv8) then
      flags = BLINK
    end
    lcd.drawText(128, 10, string.format("%04d", getValue(beacon_pref_id)), DBLSIZE + RIGHT + flags)
  end

  if gps_speed_id ~= 0 then
    lcd.drawText(1, 27, "Speed", SMLSIZE)
    lcd.drawText(72, 27, string.format("%dkts", getValue(gps_speed_id)), SMLSIZE + RIGHT)
  end

  if gps_hdg_id ~= 0 then
    lcd.drawText(80, 27, "Hdg", SMLSIZE)
    lcd.drawText(128, 27, string.format("%d", getValue(gps_hdg_id)), SMLSIZE + RIGHT + 0)
  end

  if beacon_number_of_sat_id ~= 0 then
    lcd.drawText(80, 37,  "Sat.", SMLSIZE)
    lcd.drawNumber(128, 37, getValue(beacon_number_of_sat_id), SMLSIZE + RIGHT)
  end

  if beacon_hdop_id ~= 0 then
    lcd.drawText(80, 47, "HDOP", SMLSIZE)
    lcd.drawText(128, 47, string.format("%.1f", getValue(beacon_hdop_id) / 100), SMLSIZE + RIGHT)
  end

  if beacon_frames_id ~= 0 then
    lcd.drawText(80, 57, "Sent", SMLSIZE)
    lcd.drawNumber(128, 57, getValue(beacon_frames_id), SMLSIZE + RIGHT)
  end

  if gps_gps_id ~= 0 then
    local gpsData = getValue(gps_gps_id)
    lcd.drawText(1, 37, "Lat", SMLSIZE)
    lcd.drawText(1, 47, "Lon", SMLSIZE)
    if type(gpsData) == "table" and gpsData.lat ~= nil and gpsData.lon ~= nil then
      last_lat = gpsData.lat
      last_lon = gpsData.lon
      got_gps = true
    end
    if got_gps then
      lcd.drawText(72, 37, string.format("%.5f", last_lat), SMLSIZE + RIGHT)
      lcd.drawText(72, 47, string.format("%.5f", last_lon), SMLSIZE + RIGHT)
    end
  end
  if gps_date_id ~= 0 then
    local dt = getValue(gps_date_id)
    if type(dt) == "table" then
      last_dt = dt
      got_dt = true
    end
    if got_dt then
      --lcd.drawText(1, 48, string.format("%d-%02d-%02d", dt.year, dt.mon, dt.day), SMLSIZE)
      lcd.drawText(1, 57, string.format("%02d-%02d %02d:%02d:%02dz", last_dt.mon, last_dt.day, last_dt.hour, last_dt.min, last_dt.sec), SMLSIZE)
    end
  end
end

return {init=init, background=background, run=run}

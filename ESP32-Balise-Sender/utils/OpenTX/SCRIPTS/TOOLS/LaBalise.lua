-- vim:et:sts=2:sw=2:si

local version = '0.1'
local refresh = 0
local display_refresh_needed = true
local sendPrefixState = 10 -- 0 sending once, 2 sending twice, 3..9 waiting, 10 ok
local beaconID = 0x13 -- sensorid 20
local beacon_pref_id
local selection = { selected = 1, state = false, list = {'group', 'mass', 'send'}, elements = 3 }

local prefixes = {
  group = { selected = 5, elements = 4, items = {
    {desc = '1 - Captif',  value = 1},
    {desc = '2 - Planeur', value = 2},
    {desc = '3 - Rotor',   value = 3},
    {desc = '4 - Avion',   value = 4},
    {desc = '-',           value = 0}
  }},
  mass = { selected = 6, elements = 5, items = {
    {desc = '0.8kg < m < 2kg', value =   2},
    {desc = '2kg < m < 4kg',   value =   4},
    {desc = '4kg < m <25kg',   value =  25},
    {desc = '25kg < m <150kg', value = 150},
    {desc = 'm >150kg',        value = 999},
    {desc = '-',               value =   0}
  }}
}

local function getFlags(element)
  if selection.selected ~= element then
    return 0
  end
  if selection.selected == element and selection.state == false then
    return INVERS
  end
  if selection.selected == element and selection.state == true then
    return INVERS + BLINK
  end
  return
end

local function increase(data)
  data.selected = data.selected + 1
  if data.selected > data.elements then
    data.selected = 1
  end
end

local function decrease(data)
  data.selected = data.selected - 1
  if data.selected < 1 then
    data.selected = data.elements
  end
end

local function computeSelectedPrefix()
    return prefixes.group.items[prefixes.group.selected].value * 1000 +
      prefixes.mass.items[prefixes.mass.selected].value
end

local function sendPrefix()
  if sendPrefixState < 1 then
    sportTelemetryPush(beaconID, 0x31, 0xff, computeSelectedPrefix())
    sendPrefixState = sendPrefixState + 1
  elseif sendPrefixState < 2 then
    sportTelemetryPush(beaconID, 0x31, 0xff, computeSelectedPrefix())
    sendPrefixState = sendPrefixState + 1
  elseif sendPrefixState < 10 then
    sendPrefixState = sendPrefixState + 1
    display_refresh_needed = true
  end
  ---- update lcd
  --if sendPrefixState == 3 then
  --  display_refresh_needed = true
  --end
end

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

local function init_func()
  local gv = model.getGlobalVariable(7, 0)
  for i, item in ipairs(prefixes.group.items) do
    if item.value == gv then
      prefixes.group.selected = i
    end
  end
  local gv = model.getGlobalVariable(8, 0)
  for i, item in ipairs(prefixes.mass.items) do
    if item.value == gv then
      prefixes.mass.selected = i
    end
  end
  beacon_pref_id = lookForId("Pref", "5230")
end

local function bg_func(event)
  if refresh < 5 then
    refresh = refresh + 1
    --print("refresh: ", refresh)
  end
end

local function smallDisplayRefresh()
  --print("display")
  lcd.clear()
  lcd.drawScreenTitle('LaBalise   v' .. version, 0, 0)
  lcd.drawText(1, 22, 'Prefixe', 0)
  lcd.drawText(54, 10, 'Voulu', 0)
  lcd.drawText(128, 10, 'Actuel', RIGHT)
  if beacon_pref_id ~= 0 then
    lcd.drawText(123, 22, string.format("%04d", getValue(beacon_pref_id)), RIGHT)
  else
    lcd.drawText(123, 22, "????", RIGHT + INVERS)
  end
  lcd.drawText(50, 18, string.format("%04d", computeSelectedPrefix()), DBLSIZE)
  lcd.drawText(1, 35, 'Groupe', 0)
  lcd.drawText(1, 45, 'Masse', 0)
  lcd.drawText(50, 35, prefixes.group.items[prefixes.group.selected].desc, getFlags(1))
  lcd.drawText(50, 45, prefixes.mass.items[prefixes.mass.selected].desc, getFlags(2))
  lcd.drawText(1, 56, '[Envoyer]', getFlags(3))
  if sendPrefixState < 10 then
    lcd.drawText(128, 56, 'Envoi...', INVERS + RIGHT)
  end
end

local function run_func(event)
  if refresh >= 5 or display_refresh_needed == true or selection.state == true then
    smallDisplayRefresh()
    display_refresh_needed = false
    refresh = 0
  else
    refresh = refresh + 1
  end

  -- left = up/decrease right = down/increase
  if selection.state == false then
    if event == EVT_ROT_LEFT or event == EVT_MINUS_BREAK or event == EVT_DOWN_BREAK then
      decrease(selection)
      display_refresh_needed = true
    end
    if event == EVT_ROT_RIGHT or event == EVT_PLUS_BREAK or event == EVT_UP_BREAK then
      increase(selection)
      display_refresh_needed = true
    end
  else
    if event == EVT_ROT_LEFT or event == EVT_MINUS_BREAK or event == EVT_DOWN_BREAK then
      decrease(prefixes[selection.list[selection.selected]])
      display_refresh_needed = true
    end
    if event == EVT_ROT_RIGHT or event == EVT_PLUS_BREAK or event == EVT_UP_BREAK then
      increase(prefixes[selection.list[selection.selected]])
      display_refresh_needed = true
    end
  end
  if event == EVT_ENTER_BREAK and sendPrefixState == 10 then
    if selection.selected == 1 or selection.selected == 2 then
      selection.state = not selection.state
    else -- Send button 
      if prefixes.group.selected ~= prefixes.group.elements + 1 and
         prefixes.mass.selected ~= prefixes.mass.elements + 1 then
        model.setGlobalVariable(7, 0, prefixes.group.items[prefixes.group.selected].value)
        model.setGlobalVariable(8, 0, prefixes.mass.items[prefixes.mass.selected].value)
        sendPrefixState = 0
      end
    end
    display_refresh_needed = true
  end
  if event == EVT_EXIT_BREAK then
    selection.state = false
    display_refresh_needed = true
  end
  if sportTelemetryPush() == true and sendPrefixState < 10 then
    sendPrefix()
  end
  --refresh = 0
  return 0
end

return {run=run_func, background=bg_func, init=init_func}

-- vim:et:sts=2:sw=2:si
--[[
ChgId

DanielGeA

License https://www.gnu.org/licenses/gpl-3.0.en.html

Ported from erskyTx. Thanks to MikeB

Lua script for radios X7, X9, X-lite and Horus with openTx 2.2 or higher

Change Frsky sensor Id

]]--

local version = '0.0.1'
local refresh = 0
local display_refresh_needed = true
local sendPrefixState = 3 -- 0 sending once, 1 sending twice, 2 sending third time, 3 ok
local tsReadId = 0
local tsSendId = 0
local beaconID = 0x13 -- sensorid 20
local selection = { selected = 1, state = false, list = {'group', 'mass', 'send'}, elements = 3 }

local prefixes = {
  group = { selected = 5, elements = 4, items = {
    {desc = '1 - Captif',  value = '1', int_value = 1},
    {desc = '2 - Planeur', value = '2', int_value = 2},
    {desc = '3 - Rotor',   value = '3', int_value = 3},
    {desc = '4 - Avion',   value = '4', int_value = 4},
    {desc = '-',         value = '0'}
  }},
  mass = { selected = 5, elements = 4, items = {
    {desc = '0.8kg < m < 2kg', value = '002', int_value = 2},
    {desc = '2kg < m < 4kg',   value = '004', int_value = 4},
    {desc = '4kg < m <25kg',   value = '025', int_value = 25},
    {desc = '25kg < m <150kg', value = '150', int_value = 150},
    {desc = '-', value = '000'}
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

local function sendPrefix()
  if sendPrefixState < 1 then
    sportTelemetryPush(beaconID, 0x31, 0xff, prefixes.group.items[prefixes.group.selected].int_value * 1000 + prefixes.mass.items[prefixes.mass.selected].int_value)
    sendPrefixState = sendPrefixState + 1
  elseif sendPrefixState < 2 then
    sportTelemetryPush(beaconID, 0x31, 0xff, prefixes.group.items[prefixes.group.selected].int_value * 1000 + prefixes.mass.items[prefixes.mass.selected].int_value)
    sendPrefixState = sendPrefixState + 1
  elseif sendPrefixState < 3 then
    sportTelemetryPush(beaconID, 0x31, 0xff, prefixes.group.items[prefixes.group.selected].int_value * 1000 + prefixes.mass.items[prefixes.mass.selected].int_value)
    sendPrefixState = sendPrefixState + 1
  end
  -- update lcd
  if sendPrefixState == 3 then
    display_refresh_needed = true
  end
end

local function init_func()
  local gv = model.getGlobalVariable(7, 0)
  for i, item in ipairs(prefixes.group.items) do
    if item.int_value == gv then
      prefixes.group.selected = i
    end
  end
  local gv = model.getGlobalVariable(8, 0)
  for i, item in ipairs(prefixes.mass.items) do
    if item.int_value == gv then
      prefixes.mass.selected = i
    end
  end
end

local function bg_func(event)
  if refresh < 5 then
    refresh = refresh + 1
  end
end

local function smallDisplayRefresh()
  lcd.clear()
  lcd.drawScreenTitle('LaBalise   v' .. version, 0, 0)
  lcd.drawText(1, 16, 'Prefixe', 0)
  lcd.drawText(50, 12, prefixes.group.items[prefixes.group.selected].value, DBLSIZE)
  lcd.drawText(60, 12, prefixes.mass.items[prefixes.mass.selected].value, DBLSIZE)
  lcd.drawText(1, 31, 'Groupe', 0)
  lcd.drawText(1, 41, 'Masse', 0)
  lcd.drawText(50, 31, prefixes.group.items[prefixes.group.selected].desc, getFlags(1))
  lcd.drawText(50, 41, prefixes.mass.items[prefixes.mass.selected].desc, getFlags(2))
  lcd.drawText(1, 55, '[Envoyer]', getFlags(3))
  if sendPrefixState < 3 then
    lcd.drawText(80, 55, 'Envoi...', INVERS)
  end
end

local function run_func(event)
  if refresh == 5 or display_refresh_needed == true or selection.state == true then
    smallDisplayRefresh()
    display_refresh_needed = false
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
  if event == EVT_ENTER_BREAK and sendPrefixState == 3 then
    if selection.selected == 1 or selection.selected == 2 then
      selection.state = not selection.state
    else -- Send button 
      if prefixes.group.selected ~= prefixes.group.elements + 1 and
         prefixes.mass.selected ~= prefixes.mass.elements + 1 then
        model.setGlobalVariable(7, 0, prefixes.group.items[prefixes.group.selected].int_value)
        model.setGlobalVariable(8, 0, prefixes.mass.items[prefixes.mass.selected].int_value)
        sendPrefixState = 2
      end
    end
    display_refresh_needed = true
  end
  if event == EVT_EXIT_BREAK then
    selection.state = false
    display_refresh_needed = true
  end
  if sportTelemetryPush() == true then
    if sendPrefixState < 3 then sendPrefix() end
  end
  refresh = 0
  return 0
end

return {run=run_func, background=bg_func, init=init_func}

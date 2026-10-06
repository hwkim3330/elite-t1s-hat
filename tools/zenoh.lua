-- Zenoh 1.x dissector for Wireshark / tshark (Lua), written against zenoh-pico 1.10.1's codec.
--
-- Covers what a zenoh-pico peer puts on a UDP multicast locator: JOIN, KEEP_ALIVE, FRAME and
-- FRAGMENT at the transport layer; inside a FRAME the network messages PUSH (PUT / DEL with
-- timestamp, encoding, extensions, payload) and DECLARE (key expressions, subscribers,
-- queryables, tokens, final); REQUEST / RESPONSE / INTEREST / OAM are named and skipped.
--
-- Install: copy to ~/.local/lib/wireshark/plugins/ (both wireshark and tshark load it), or run
--   tshark -X lua_script:zenoh.lua -r capture.pcap
-- Registered on UDP 7447 (Zenoh's default). Decode As... "ZENOH" for other ports.
--
-- Wire format (zenoh-pico src/protocol/codec): every message starts with a header byte,
-- message id in bits 0-4 and flags in 5-7; integers are LEB128 ("zint"); extensions are
-- [hdr: id 0-3, M 4, enc 5-6 (unit / zint / zbuf), Z(more) 7]. A UDP datagram holds whole
-- transport messages back to back, with no length prefix (that exists only on stream links).

local zenoh = Proto("zenoh", "Zenoh")

local T_NAMES = {[0x00] = "OAM", [0x01] = "INIT", [0x02] = "OPEN", [0x03] = "CLOSE", [0x04] = "KEEP_ALIVE",
                 [0x05] = "FRAME", [0x06] = "FRAGMENT", [0x07] = "JOIN"}
local N_NAMES = {[0x1f] = "OAM", [0x1e] = "DECLARE", [0x1d] = "PUSH", [0x1c] = "REQUEST", [0x1b] = "RESPONSE",
                 [0x1a] = "RESPONSE_FINAL", [0x19] = "INTEREST"}
local Z_NAMES = {[0x01] = "PUT", [0x02] = "DEL", [0x03] = "QUERY", [0x04] = "REPLY", [0x05] = "ERR"}
local D_NAMES = {[0x00] = "D_KEYEXPR", [0x01] = "U_KEYEXPR", [0x02] = "D_SUBSCRIBER", [0x03] = "U_SUBSCRIBER",
                 [0x04] = "D_QUERYABLE", [0x05] = "U_QUERYABLE", [0x06] = "D_TOKEN", [0x07] = "U_TOKEN",
                 [0x1a] = "D_FINAL"}
local WHATAMI = {[0] = "router", [1] = "peer", [2] = "client"}

local f = zenoh.fields
f.t_msg = ProtoField.string("zenoh.transport", "Transport message")
f.t_mid = ProtoField.uint8("zenoh.t_mid", "Transport id", base.HEX, nil, 0x1f)
f.flags = ProtoField.uint8("zenoh.flags", "Flags", base.HEX, nil, 0xe0)
f.version = ProtoField.uint8("zenoh.version", "Protocol version", base.HEX)
f.whatami = ProtoField.string("zenoh.whatami", "WhatAmI")
f.zid = ProtoField.bytes("zenoh.zid", "Zenoh ID")
f.lease = ProtoField.uint64("zenoh.lease", "Lease")
f.batch = ProtoField.uint16("zenoh.batch_size", "Batch size")
f.sn = ProtoField.uint64("zenoh.sn", "Sequence number")
f.sn_be = ProtoField.uint64("zenoh.sn_be", "Next SN best-effort")
f.reliable = ProtoField.bool("zenoh.reliable", "Reliable")
f.n_msg = ProtoField.string("zenoh.network", "Network message")
f.key = ProtoField.string("zenoh.key", "Key expression")
f.key_id = ProtoField.uint64("zenoh.key_id", "Key expression id")
f.z_msg = ProtoField.string("zenoh.msg", "Zenoh message")
f.ts = ProtoField.string("zenoh.timestamp", "Timestamp")
f.encoding = ProtoField.uint64("zenoh.encoding", "Encoding id")
f.payload = ProtoField.bytes("zenoh.payload", "Payload")
f.payload_txt = ProtoField.string("zenoh.payload_text", "Payload (text)")
f.decl = ProtoField.string("zenoh.decl", "Declaration")
f.decl_id = ProtoField.uint64("zenoh.decl_id", "Declaration id")
f.ext = ProtoField.string("zenoh.ext", "Extension")
f.interest_id = ProtoField.uint64("zenoh.interest_id", "Interest id")

local ef_short = ProtoExpert.new("zenoh.short", "Message runs past the end of the datagram", expert.group.MALFORMED, expert.severity.ERROR)
zenoh.experts = {ef_short}

-- key expression ids declared by D_KEYEXPR, so later PUSHes that send only an id get a name
local declared = {}

local function zint(tvb, off)
  local v, shift, i = UInt64(0), 0, 0
  while true do
    if off + i >= tvb:len() then error("short") end
    local b = tvb(off + i, 1):uint()
    v = v + UInt64(bit.band(b, 0x7f)):lshift(shift)
    i = i + 1
    if bit.band(b, 0x80) == 0 or i == 9 then break end
    shift = shift + 7
  end
  return v, i
end

local function zbytes(tvb, off)        -- zint length + bytes
  local n, l = zint(tvb, off)
  n = n:tonumber()
  if off + l + n > tvb:len() then error("short") end
  return tvb(off + l, n), l + n
end

local function printable(r)
  if r:len() == 0 then return "" end
  local s = r:raw()
  if s:match("^[%g ]+$") then return s end
  return nil
end

local function extensions(tvb, off, tree)
  local start = off
  while true do
    local h = tvb(off, 1):uint()
    local enc, id = bit.band(bit.rshift(h, 5), 3), bit.band(h, 0x0f)
    local len = 1
    if enc == 1 then local _, l = zint(tvb, off + 1); len = len + l
    elseif enc == 2 then local _, l = zbytes(tvb, off + 1); len = len + l end
    tree:add(f.ext, tvb(off, len), string.format("id %d, %s%s", id, ({"unit", "zint", "zbuf", "?"})[enc + 1],
                                                 bit.band(h, 0x10) ~= 0 and ", mandatory" or ""))
    off = off + len
    if bit.band(h, 0x80) == 0 then break end
  end
  return off - start
end

local function wireexpr(tvb, off, has_suffix, tree)
  local id, l = zint(tvb, off)
  local name, len = nil, l
  if has_suffix then
    local s, sl = zbytes(tvb, off + l)
    name = s:string()
    len = len + sl
  end
  local idn = id:tonumber()
  local full = (idn ~= 0 and declared[idn] or "") .. (name or "")
  if full == "" then full = "#" .. idn end
  local it = tree:add(f.key, tvb(off, len), full)
  if idn ~= 0 then it:append_text(string.format("  (id %d%s)", idn, name and " + suffix" or "")) end
  return full, len, idn, name
end

local function push_body(tvb, off, tree, info)
  local h = tvb(off, 1):uint()
  local mid = bit.band(h, 0x1f)
  local name = Z_NAMES[mid] or string.format("0x%02x", mid)
  local st = tree:add(f.z_msg, tvb(off, 1), name)
  local o = off + 1
  if bit.band(h, 0x20) ~= 0 then   -- T: timestamp = zint time + id slice
    local t, l = zint(tvb, o)
    local n = tvb(o + l, 1):uint()
    st:add(f.ts, tvb(o, l + 1 + n), tostring(t))
    o = o + l + 1 + n
  end
  if mid == 0x01 and bit.band(h, 0x40) ~= 0 then   -- E: encoding = zint (id << 1 | has schema) [+ schema]
    local e, l = zint(tvb, o)
    local en = e:tonumber()
    st:add(f.encoding, tvb(o, l), math.floor(en / 2))
    o = o + l
    if en % 2 == 1 then local _, sl = zbytes(tvb, o); o = o + sl end
  end
  if bit.band(h, 0x80) ~= 0 then o = o + extensions(tvb, o, st) end
  local preview = ""
  if mid == 0x01 then
    local p, l = zbytes(tvb, o)
    if p:len() > 0 then st:add(f.payload, p) end
    local txt = printable(p)
    if txt then st:add(f.payload_txt, p, txt); preview = " '" .. (#txt > 24 and txt:sub(1, 24) .. "…" or txt) .. "'"
    else preview = string.format(" (%d B)", p:len()) end
    o = o + l
  end
  st:set_len(o - off)
  table.insert(info, name .. preview)
  return o - off
end

local function network(tvb, off, tree, info)
  local h = tvb(off, 1):uint()
  local mid = bit.band(h, 0x1f)
  local name = N_NAMES[mid] or string.format("N 0x%02x", mid)
  local nt = tree:add(f.n_msg, tvb(off, 1), name)
  local o = off + 1
  if mid == 0x1d then   -- PUSH
    local key, l = wireexpr(tvb, o, bit.band(h, 0x20) ~= 0, nt)
    o = o + l
    if bit.band(h, 0x80) ~= 0 then o = o + extensions(tvb, o, nt) end
    local sub = {}
    o = o + push_body(tvb, o, nt, sub)
    nt:append_text(" " .. key)
    table.insert(info, (sub[1]:gsub("^(%u+)", "%1 " .. key, 1)))
  elseif mid == 0x1e then   -- DECLARE
    if bit.band(h, 0x20) ~= 0 then local v, l = zint(tvb, o); nt:add(f.interest_id, tvb(o, l), v); o = o + l end
    if bit.band(h, 0x80) ~= 0 then o = o + extensions(tvb, o, nt) end
    local dh = tvb(o, 1):uint()
    local dmid = bit.band(dh, 0x1f)
    local dname = D_NAMES[dmid] or string.format("decl 0x%02x", dmid)
    local dt = nt:add(f.decl, tvb(o, 1), dname)
    local p = o + 1
    local what = dname
    if dmid == 0x1a then
      -- D_FINAL: no body
    else
      local id, l = zint(tvb, p)
      dt:add(f.decl_id, tvb(p, l), id)
      p = p + l
      if dmid == 0x00 or dmid == 0x02 or dmid == 0x04 or dmid == 0x06 then
        local key, kl, kid, suffix = wireexpr(tvb, p, bit.band(dh, 0x20) ~= 0, dt)
        p = p + kl
        if dmid == 0x00 then declared[id:tonumber()] = (kid ~= 0 and declared[kid] or "") .. (suffix or "") end
        what = what .. " " .. key
      end
      if bit.band(dh, 0x80) ~= 0 then p = p + extensions(tvb, p, dt) end
    end
    dt:set_len(p - o)
    o = p
    nt:append_text(" " .. what)
    table.insert(info, what)
  else
    -- REQUEST / RESPONSE / INTEREST / OAM: named, not decoded; they run to the end of the frame
    o = tvb:len()
    table.insert(info, name)
  end
  nt:set_len(o - off)
  return o - off
end

function zenoh.dissector(tvb, pinfo, tree)
  if tvb:len() == 0 then return 0 end
  pinfo.cols.protocol = "Zenoh"
  local root = tree:add(zenoh, tvb())
  local info, off = {}, 0
  local ok, err = pcall(function()
    while off < tvb:len() do
      local h = tvb(off, 1):uint()
      local mid = bit.band(h, 0x1f)
      local name = T_NAMES[mid] or string.format("T 0x%02x", mid)
      local tt = root:add(f.t_msg, tvb(off, 1), name)
      tt:add(f.t_mid, tvb(off, 1))
      tt:add(f.flags, tvb(off, 1))
      local o = off + 1
      if mid == 0x07 then   -- JOIN
        tt:add(f.version, tvb(o, 1)); o = o + 1
        local c = tvb(o, 1):uint()
        local zl = bit.rshift(c, 4) + 1
        tt:add(f.whatami, tvb(o, 1), WHATAMI[bit.band(c, 3)] or tostring(bit.band(c, 3)))
        tt:add(f.zid, tvb(o + 1, zl))
        local zid = tvb(o + 1, zl):bytes():tohex():lower()
        o = o + 1 + zl
        if bit.band(h, 0x40) ~= 0 then tt:add_le(f.batch, tvb(o + 1, 2)); o = o + 3 end
        local lease, l = zint(tvb, o)
        tt:add(f.lease, tvb(o, l), lease):append_text(bit.band(h, 0x20) ~= 0 and " s" or " ms"); o = o + l
        local sn, l2 = zint(tvb, o); tt:add(f.sn, tvb(o, l2), sn); o = o + l2
        local sb, l3 = zint(tvb, o); tt:add(f.sn_be, tvb(o, l3), sb); o = o + l3
        if bit.band(h, 0x80) ~= 0 then o = o + extensions(tvb, o, tt) end
        table.insert(info, string.format("JOIN %s %s", WHATAMI[bit.band(c, 3)] or "?", zid))
      elseif mid == 0x05 or mid == 0x06 then   -- FRAME / FRAGMENT
        tt:add(f.reliable, tvb(off, 1), bit.band(h, 0x20) ~= 0)
        local sn, l = zint(tvb, o)
        tt:add(f.sn, tvb(o, l), sn)
        o = o + l
        if bit.band(h, 0x80) ~= 0 then o = o + extensions(tvb, o, tt) end
        if mid == 0x05 then
          local n = 0
          while o < tvb:len() do
            o = o + network(tvb, o, tt, info)
            n = n + 1
          end
          if n == 0 then table.insert(info, "FRAME (empty)") end
        else
          tt:add(f.payload, tvb(o))
          table.insert(info, string.format("FRAGMENT sn %s%s (%d B)", tostring(sn),
                                           bit.band(h, 0x40) ~= 0 and " more" or " last", tvb:len() - o))
          o = tvb:len()
        end
      elseif mid == 0x04 then   -- KEEP_ALIVE
        if bit.band(h, 0x80) ~= 0 then o = o + extensions(tvb, o, tt) end
        table.insert(info, "KEEP_ALIVE")
      else
        o = tvb:len()   -- INIT / OPEN / CLOSE / OAM: unicast session setup, not decoded
        table.insert(info, name)
      end
      tt:set_len(o - off)
      off = o
    end
  end)
  if not ok then root:add_proto_expert_info(ef_short, tostring(err)) end
  pinfo.cols.info = table.concat(info, ", ")
  return tvb:len()
end

DissectorTable.get("udp.port"):add(7447, zenoh)

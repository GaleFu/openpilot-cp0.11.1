def get_selected_car_platform(name: str):
  from opendbc.car.changan.values import CAR as CHANGAN
  from opendbc.car.ford.values import CAR as FORD
  from opendbc.car.hyundai.values import CAR as HYUNDAI
  from opendbc.car.gm.values import CAR as GM
  from opendbc.car.toyota.values import CAR as TOYOTA
  from opendbc.car.mazda.values import CAR as MAZDA
  from opendbc.car.volkswagen.values import CAR as VOLKSWAGEN
  from opendbc.car.tesla.values import CAR as TESLA

  selected = str(name or "").strip().lower()
  aliases = {
    "changan oushang z6": CHANGAN.CHANGAN_Z6,
    "oushang z6": CHANGAN.CHANGAN_Z6,
    "\u957f\u5b89 \u6b27\u5c1a z6": CHANGAN.CHANGAN_Z6,
    "changan z6 idd": CHANGAN.CHANGAN_Z6_IDD,
    "changan oushang z6 idd": CHANGAN.CHANGAN_Z6_IDD,
    "oushang z6 idd": CHANGAN.CHANGAN_Z6_IDD,
    "\u957f\u5b89 \u6b27\u5c1a z6 idd": CHANGAN.CHANGAN_Z6_IDD,
  }
  if selected in aliases:
    return aliases[selected]

  platforms = [platform for brand in (FORD, GM, TOYOTA, HYUNDAI, MAZDA, VOLKSWAGEN, CHANGAN) for platform in brand]
  # Model X is intentionally dashcam-only. Model 3/Y have a CarController and
  # must be selectable even when automatic fingerprinting is unavailable.
  platforms.extend((TESLA.TESLA_MODEL_3, TESLA.TESLA_MODEL_Y))

  return next(
    (platform for platform in platforms
     for doc in platform.config.car_docs
     if selected == str(doc.name).strip().lower()),
    None,
  )

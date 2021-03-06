-- LI3DS
CREATE EXTENSION postgis;
CREATE EXTENSION pointcloud_postgis;

INSERT INTO pointcloud_formats (pcid, srid, schema)
VALUES (3, 0, -- XYZI, scaled, dimensionally compressed
'<?xml version="1.0" encoding="UTF-8"?>
<pc:PointCloudSchema xmlns:pc="http://pointcloud.org/schemas/PC/1.1" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <pc:dimension>
    <pc:position>1</pc:position>
    <pc:size>4</pc:size>
    <pc:description>X coordinate as a long integer. You must use the scale and offset information of the header to determine the double value.</pc:description>
    <pc:name>X</pc:name>
    <pc:interpretation>int32_t</pc:interpretation>
    <pc:scale>0.01</pc:scale>
  </pc:dimension>
  <pc:dimension>
    <pc:position>2</pc:position>
    <pc:size>4</pc:size>
    <pc:description>Y coordinate as a long integer. You must use the scale and offset information of the header to determine the double value.</pc:description>
    <pc:name>Y</pc:name>
    <pc:interpretation>int32_t</pc:interpretation>
    <pc:scale>0.01</pc:scale>
  </pc:dimension>
  <pc:dimension>
    <pc:position>3</pc:position>
    <pc:size>4</pc:size>
    <pc:description>Z coordinate as a long integer. You must use the scale and offset information of the header to determine the double value.</pc:description>
    <pc:name>Z</pc:name>
    <pc:interpretation>int32_t</pc:interpretation>
    <pc:scale>0.01</pc:scale>
  </pc:dimension>
  <pc:dimension>
    <pc:position>4</pc:position>
    <pc:size>2</pc:size>
    <pc:description>The intensity value is the integer representation of the pulse return magnitude. This value is optional and system specific. However, it should always be included if available.</pc:description>
    <pc:name>Intensity</pc:name>
    <pc:interpretation>uint16_t</pc:interpretation>
    <pc:scale>1</pc:scale>
  </pc:dimension>
  <pc:metadata>
    <Metadata name="compression">dimensional</Metadata>
    <Metadata name="spatialreference" type="id">4326</Metadata>
  </pc:metadata>
</pc:PointCloudSchema>'
);

CREATE TABLE IF NOT EXISTS pa_test_frustum_li3ds (
    pa PCPATCH(3)
);
\d pa_test_frustum_li3ds

INSERT INTO pa_test_frustum_li3ds (pa)
SELECT PC_Patch(PC_MakePoint(3, ARRAY[x,y,z,intensity]))
FROM (
 SELECT
 -127+a/100.0 AS x,
   45+a/100.0 AS y,
        1.0*a AS z,
         a/10 AS intensity,
         a/400 AS gid
 FROM generate_series(1,1600) AS a
) AS values GROUP BY gid;

SELECT ST_AsText(PC_EnvelopeGeometry(pa)), ST_AsText(PC_FrustumAsGeom(pa)) FROM pa_test_frustum_li3ds;

DROP TABLE pa_test_frustum_li3ds;

SELECT 'BOX4D(1.0 1.0 1.0 1.0,2.0 2.0 2.0 2.0)'::LIBOX4D;
SELECT 'BOX4D(1.0 1.0 1.0,2.0 2.0 2.0)'::LIBOX4D;
SELECT 'BOX4D(1.0 1.0,2.0 2.0)'::LIBOX4D;

SELECT PC_Affine('BOX4D(1 1 1 1,4 4 4 4)'::LIBOX4D, ARRAY[2,0,0,1,0,1,0,1,0,0,1,1]);
SELECT PC_Affine('BOX4D(-1 -1 1 10,1 1 -1 20)'::LIBOX4D, ARRAY[cos(pi()/8.0),sin(pi()/8.0),0,0], ARRAY[1,0,0]);

TRUNCATE pointcloud_formats;

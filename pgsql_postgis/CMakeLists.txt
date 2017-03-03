
configure_file(
    pointcloud_postgis.sql.in
    "${PROJECT_BINARY_DIR}/pgsql/pointcloud_postgis--${POINTCLOUD_VERSION}.sql"
  )

configure_file(
    pointcloud_postgis.control.in
    "${PROJECT_BINARY_DIR}/pgsql/pointcloud_postgis.control"
  )

set ( PCPG_INSTALL_EXTENSIONS
    "${PROJECT_BINARY_DIR}/pgsql/pointcloud_postgis--${POINTCLOUD_VERSION}.sql"
    "${PROJECT_BINARY_DIR}/pgsql/pointcloud_postgis.control"
  )

install (
  FILES ${PCPG_INSTALL_EXTENSIONS}
  DESTINATION "${PGSQL_SHAREDIR}/extension"
  )


awk -i inplace 'BEGIN{FS=OFS="."} {if ($0 ~ /[0-9]+.[0-9]+.[0-9]+/) {$3+=1; print $1"."$2"."$3; next}} 1' version.txtCLASSFILE=$(cat << EOF


class version {
        public string version = "$VERSION"
}
EOF

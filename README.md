# UHRY_MUGNIER_Info_Indus_projet


python3 -m venv .venv
source .venv/bin/activate

sphinx-build -M html docs/source docs/_build
rm -f docs/*.html docs/*.js docs/*.inv
rm -rf docs/_static docs/_sources
cp -R docs/_build/html/* docs/
git add docs
git commit -m "MAJ doc"
git push

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

public class RobotPreferenceFactory {

  public static <E extends Enum<E>> PreferenceFloat<E> createPreferenceFloat(Enum<E> id, String ntPath, float defaultValue) {
    return new PreferenceFloat<E>(id, ntPath, defaultValue);
  }

  public static <E extends Enum<E>> PreferenceDouble<E> createPreferenceDouble(Enum<E> id, String ntPath, double defaultValue) {
    return new PreferenceDouble<E>(id, ntPath, defaultValue);
  }

  public static <E extends Enum<E>> PreferenceInt<E> createPreferenceInt(Enum<E> id, String ntPath, int defaultValue) {
    return new PreferenceInt<E>(id, ntPath, defaultValue);
  }

  public static <E extends Enum<E>> PreferenceBoolean<E> createPreferenceBoolean(Enum<E> id, String ntPath, boolean defaultValue) {
    return new PreferenceBoolean<E>(id, ntPath, defaultValue);
  }

  public static <E extends Enum<E>> PreferenceLong<E> createPreferenceLong(Enum<E> id, String ntPath, long defaultValue) {
    return new PreferenceLong<E>(id, ntPath, defaultValue);
  }

  public static <E extends Enum<E>> PreferenceString<E> createPreferenceString(Enum<E> id, String ntPath, String defaultValue) {
    return new PreferenceString<E>(id, ntPath, defaultValue);
  }

  public static class PreferenceFloat<E extends Enum<E>> {
    Enum<E> id;
    String ntPath;
    float defaultValue;

    protected PreferenceFloat(Enum<E> id, String ntPath, float defaultValue) {
      this.id = id;
      this.ntPath = ntPath;
      this.defaultValue = defaultValue;
      Preferences.initFloat(ntPath + "/" + id.name(), defaultValue);
    }

    public float Value() {
      return Preferences.getFloat(ntPath + "/" + id.name(), defaultValue);
    }

    public void setValue(float value) {
      Preferences.setFloat(ntPath + "/" + id.name(), value);
    }
  }

  public static class PreferenceDouble<E extends Enum<E>> {
    Enum<E> id;
    String ntPath;
    double defaultValue;

    protected PreferenceDouble(Enum<E> id, String ntPath, double defaultValue) {
      this.id = id;
      this.ntPath = ntPath;
      this.defaultValue = defaultValue;
      Preferences.initDouble(ntPath + "/" + id.name(), defaultValue);
    }

    public double Value() {
      return Preferences.getDouble(ntPath + "/" + id.name(), defaultValue);
    }

    public void setValue(double value) {
      Preferences.setDouble(ntPath + "/" + id.name(), value);
    }
  }

  public static class PreferenceInt<E extends Enum<E>> {
    Enum<E> id;
    String ntPath;
    int defaultValue;

    protected PreferenceInt(Enum<E> id, String ntPath, int defaultValue) {
      this.id = id;
      this.ntPath = ntPath;
      this.defaultValue = defaultValue;
      Preferences.initInt(ntPath + "/" + id.name(), defaultValue);
    }

    public int Value() {
      return Preferences.getInt(ntPath + "/" + id.name(), defaultValue);
    }

    public void setValue(int value) {
      Preferences.setInt(ntPath + "/" + id.name(), value);
    }
  }

  public static class PreferenceBoolean<E extends Enum<E>> {
    Enum<E> id;
    String ntPath;
    boolean defaultValue;

    protected PreferenceBoolean(Enum<E> id, String ntPath, boolean defaultValue) {
      this.id = id;
      this.ntPath = ntPath;
      this.defaultValue = defaultValue;
      Preferences.initBoolean(ntPath + "/" + id.name(), defaultValue);
    }

    public boolean Value() {
      return Preferences.getBoolean(ntPath + "/" + id.name(), defaultValue);
    }

    public void setValue(boolean value) {
      Preferences.setBoolean(ntPath + "/" + id.name(), value);
    }
  }

  public static class PreferenceLong<E extends Enum<E>> {
    Enum<E> id;
    String ntPath;
    long defaultValue;

    protected PreferenceLong(Enum<E> id, String ntPath, long defaultValue) {
      this.id = id;
      this.ntPath = ntPath;
      this.defaultValue = defaultValue;
      Preferences.initLong(ntPath + "/" + id.name(), defaultValue);
    }

    public long Value() {
      return Preferences.getLong(ntPath + "/" + id.name(), defaultValue);
    }

    public void setValue(long value) {
      Preferences.setLong(ntPath + "/" + id.name(), value);
    }
  }

  public static class PreferenceString<E extends Enum<E>> {
    Enum<E> id;
    String ntPath;
    String defaultValue;

    protected PreferenceString(Enum<E> id, String ntPath, String defaultValue) {
      this.id = id;
      this.ntPath = ntPath;
      this.defaultValue = defaultValue;
      Preferences.initString(ntPath + "/" + id.name(), defaultValue);
    }

    public String Value() {
      return Preferences.getString(ntPath + "/" + id.name(), defaultValue);
    }

    public void setValue(String value) {
      Preferences.setString(ntPath + "/" + id.name(), value);
    }
  }
}


